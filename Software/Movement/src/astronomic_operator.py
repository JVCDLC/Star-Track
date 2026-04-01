import copy
import json
import math
import re
import time
from collections import deque
from datetime import datetime, timedelta, timezone
from enum import Enum
from pathlib import Path
from threading import Event, Lock, Thread

try:
    import serial
except Exception:
    serial = None

try:
    from skyfield.api import Loader, Star, wgs84
    from skyfield.data import hipparcos
except Exception:
    Loader = None
    Star = None
    wgs84 = None
    hipparcos = None

try:
    from skyfield import almanac
except Exception:
    almanac = None


def _clamp(value, low, high):
    return max(low, min(high, value))


def _parse_datetime_to_ms(value):
    if not value:
        return None
    try:
        normalized = str(value).replace("Z", "+00:00")
        return int(datetime.fromisoformat(normalized).timestamp() * 1000)
    except Exception:
        return None


class MotorLink:
    """
    Arduino serial bridge.
    Incoming numeric responses:
    - 0: rejected
    - 1: accepted/moving
    - 2: done/idle
    """

    def __init__(
        self,
        simulate=True,
        port="/dev/ttyUSB0",
        baudrate=115200,
        log_file=None,
        log_max=4000,
    ):
        self.simulate = bool(simulate)
        self.port = port
        self.baudrate = int(baudrate)
        self.serial_conn = None
        self._rx_buffer = bytearray()

        self.lock = Lock()
        self.shutdown = Event()
        self.rx_thread = None

        self.last_error = ""
        self.last_tx_at = 0.0
        self.last_rx_at = 0.0
        self.last_response = ""
        self.last_debug_line = ""
        self.system_state = "IDLE"
        self.calibration_done = False
        self.motor_state = {"RA": "IDLE", "DEC": "IDLE", "FOC": "IDLE"}
        self.motor_position = {"RA": None, "DEC": None, "FOC": None}
        self.motor_target = {"RA": None, "DEC": None, "FOC": None}

        self.pending_tx = deque(maxlen=200)
        self.last_commands = deque(maxlen=300)
        self.responses = deque(maxlen=300)
        self.events = deque(maxlen=300)
        self._inflight_by_motor = {}
        self._inflight_global = None
        self.serial_exchange = deque(maxlen=max(200, int(log_max)))
        self.serial_log_file = Path(log_file) if log_file else None
        self.sim_pending_done = deque(maxlen=100)
        self.sim_steps = {"RA": 0.0, "DEC": 0.0, "FOC": 0.0}
        if self.serial_log_file is not None:
            try:
                self.serial_log_file.parent.mkdir(parents=True, exist_ok=True)
                self.serial_log_file.touch(exist_ok=True)
            except Exception as exc:
                self.last_error = str(exc)
                self.serial_log_file = None

        if not self.simulate and serial is not None:
            try:
                self.serial_conn = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=0.2,
                    write_timeout=0.2,
                )
                self.rx_thread = Thread(target=self._rx_loop, daemon=True)
                self.rx_thread.start()
            except Exception as exc:
                self.last_error = str(exc)
                self.simulate = True
        elif not self.simulate and serial is None:
            self.last_error = "pyserial not installed. Falling back to simulation."
            self.simulate = True

    def _build_cmd(self, action, motor=None, value="", value2=""):
        parts = [str(int(action))]
        if motor not in (None, ""):
            parts.append(str(motor).upper())
        if value != "":
            parts.append(str(value))
        if value2 != "":
            parts.append(str(value2))
        return ",".join(parts)

    def send_command(self, action, motor=None, value="", value2=""):
        cmd = self._build_cmd(action, motor, value, value2)
        return self.send_raw(cmd, action=action, motor=motor)

    def send_raw(self, command, action=None, motor=None):
        now = time.time()
        meta = None
        with self.lock:
            meta = {
                "ts": now,
                "cmd": str(command),
                "action": int(action) if action is not None else None,
                "motor": str(motor).upper() if motor else None,
            }
            self.last_tx_at = now
            self.pending_tx.append(meta)
            self.last_commands.append(meta)
            self._record_serial_exchange_locked(
                direction="tx",
                line=str(command),
                kind="cmd",
                code=None,
                meta=meta,
            )

            if not self.simulate:
                try:
                    self.serial_conn.write((str(command) + "\n").encode("utf-8"))
                    return True
                except Exception as exc:
                    self.last_error = str(exc)
                    self._record_serial_exchange_locked(
                        direction="tx",
                        line=str(command),
                        kind="error",
                        code=None,
                        meta={**meta, "error": str(exc)},
                    )
                    return False

        if self.simulate:
            self._on_response_line("1", meta_override=meta)
            if meta["action"] in (1, 2, 3, 4, 0):
                delay = 1.0 if meta["action"] in (3, 4) else 0.05
                if meta["action"] in (1, 2) and meta.get("motor") in self.sim_steps:
                    try:
                        parts = str(meta["cmd"]).split(",")
                        target = float(parts[2])
                    except Exception:
                        target = float(self.sim_steps.get(meta["motor"], 0.0))
                    start = float(self.sim_steps.get(meta["motor"], 0.0))
                    distance = abs(target - start)
                    if meta["action"] == 2 and len(str(meta["cmd"]).split(",")) >= 4:
                        try:
                            speed = max(1.0, abs(float(str(meta["cmd"]).split(",")[3])))
                        except Exception:
                            speed = 500.0
                        delay = max(0.02, distance / speed)
                    elif meta["action"] == 1:
                        delay = max(0.02, distance / 1800.0)
                    self.sim_steps[meta["motor"]] = target
                with self.lock:
                    self.sim_pending_done.append((now + delay, meta))
            return True

        return False

    def _rx_loop(self):
        while not self.shutdown.is_set():
            try:
                waiting = 0
                try:
                    waiting = int(getattr(self.serial_conn, "in_waiting", 0) or 0)
                except Exception:
                    waiting = 0
                chunk = self.serial_conn.read(waiting if waiting > 0 else 1)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                time.sleep(0.05)
                continue
            if chunk:
                self._process_rx_bytes(chunk)
        # Flush trailing buffered bytes at shutdown (best-effort).
        if self._rx_buffer:
            tail = bytes(self._rx_buffer).decode("utf-8", errors="replace").strip()
            self._rx_buffer.clear()
            if tail:
                self._on_response_line(tail)

    def _process_rx_bytes(self, data):
        if not data:
            return
        self._rx_buffer.extend(data)
        while True:
            nl_index = self._rx_buffer.find(b"\n")
            if nl_index < 0:
                # Guardrail: avoid unbounded growth if firmware emits malformed output.
                if len(self._rx_buffer) > 8192:
                    line = bytes(self._rx_buffer).decode("utf-8", errors="replace").strip()
                    self._rx_buffer.clear()
                    if line:
                        self._on_response_line(line)
                return
            raw = bytes(self._rx_buffer[:nl_index])
            del self._rx_buffer[: nl_index + 1]
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self._on_response_line(line)

    def _apply_code(self, code, meta):
        motor = (meta or {}).get("motor")
        action = (meta or {}).get("action")
        if code == 0:
            if motor in self.motor_state:
                self.motor_state[motor] = "ERROR"
            self.system_state = "ERROR"
            return
        if code == 1:
            if action in (3, 4):
                self.system_state = "CALIBRATING"
                self.calibration_done = False
            elif motor in self.motor_state:
                self.motor_state[motor] = "MOVING"
                self.system_state = "MOVING"
            return
        if code == 2:
            if action in (3, 4):
                self.calibration_done = True
                self.system_state = "READY"
                self.motor_state = {"RA": "IDLE", "DEC": "IDLE", "FOC": "IDLE"}
            elif motor in self.motor_state:
                self.motor_state[motor] = "IDLE"
                if all(v == "IDLE" for v in self.motor_state.values()):
                    self.system_state = "READY"
            else:
                self.system_state = "READY"

    def _on_response_line(self, line, meta_override=None):
        now = time.time()
        with self.lock:
            self.last_rx_at = now
            self.last_response = str(line)
            code = None
            parsed_motor = None
            parsed_position = None
            raw_line = str(line).strip()
            parts = [part.strip() for part in raw_line.split(",")] if raw_line else []
            try:
                if parts:
                    code = int(parts[0])
                else:
                    code = int(raw_line)
            except Exception:
                code = None
            if len(parts) >= 2:
                token = str(parts[1]).upper()
                if token in {"RA", "DEC", "FOC"}:
                    parsed_motor = token
            if len(parts) >= 3:
                try:
                    parsed_position = int(round(float(parts[2])))
                except Exception:
                    parsed_position = None

            if code in (0, 1, 2):
                base_meta = meta_override or (self.pending_tx.popleft() if self.pending_tx else None)
                if code == 2 and base_meta is None:
                    if parsed_motor in self._inflight_by_motor:
                        base_meta = self._inflight_by_motor.pop(parsed_motor)
                    elif self._inflight_global is not None:
                        base_meta = self._inflight_global
                        self._inflight_global = None
                resolved_meta = {
                    **(base_meta or {}),
                    "motor": (base_meta or {}).get("motor") or parsed_motor,
                    "position": parsed_position,
                }
                resolved_motor = str((resolved_meta or {}).get("motor") or "").upper()
                if code == 1:
                    if resolved_motor in {"RA", "DEC", "FOC"}:
                        if (
                            (resolved_meta or {}).get("action") is None
                            and resolved_motor in self._inflight_by_motor
                        ):
                            preserved = dict(self._inflight_by_motor[resolved_motor])
                            if (resolved_meta or {}).get("position") is not None:
                                preserved["position"] = (resolved_meta or {}).get("position")
                            self._inflight_by_motor[resolved_motor] = preserved
                        else:
                            self._inflight_by_motor[resolved_motor] = dict(resolved_meta)
                    elif (resolved_meta or {}).get("action") in (3, 4):
                        self._inflight_global = dict(resolved_meta)
                elif code in (0, 2):
                    if resolved_motor in self._inflight_by_motor:
                        self._inflight_by_motor.pop(resolved_motor, None)
                    if (resolved_meta or {}).get("action") in (3, 4):
                        self._inflight_global = None
                ev = {
                    "ts": now,
                    "kind": "ack",
                    "code": code,
                    "line": str(line),
                    "cmd": (resolved_meta or {}).get("cmd"),
                    "action": (resolved_meta or {}).get("action"),
                    "motor": (resolved_meta or {}).get("motor"),
                    "position": parsed_position,
                }
                self.responses.append(ev)
                self.events.append(ev)
                self._record_serial_exchange_locked(
                    direction="rx",
                    line=str(line),
                    kind="ack",
                    code=code,
                    meta=resolved_meta,
                )
                self._apply_code(code, resolved_meta)
            else:
                self.last_debug_line = str(line)
                self._parse_debug_snapshot_locked(str(line))
                self.events.append({"ts": now, "kind": "debug", "line": str(line)})
                self._record_serial_exchange_locked(
                    direction="rx",
                    line=str(line),
                    kind="debug",
                    code=None,
                    meta=meta_override,
                )

    def _flush_sim(self):
        if not self.simulate:
            return
        now = time.time()
        while self.sim_pending_done and self.sim_pending_done[0][0] <= now:
            _, meta = self.sim_pending_done.popleft()
            self._on_response_line("2", meta_override=meta)

    def _parse_debug_snapshot_locked(self, line):
        text = str(line or "")
        if not text:
            return
        try:
            mount_match = re.search(r"\bmount=([A-Za-z_]+)", text, flags=re.IGNORECASE)
            if mount_match:
                self.system_state = str(mount_match.group(1)).upper()
        except Exception:
            pass
        for motor in ("RA", "DEC", "FOC"):
            try:
                pattern = (
                    rf"\b{motor}\(([^)]+)\)\s*pos=([-+]?\d+)"
                    rf"(?:\s*tgt=([-+]?\d+))?"
                )
                match = re.search(pattern, text, flags=re.IGNORECASE)
                if not match:
                    continue
                state_txt = str(match.group(1)).strip().upper()
                self.motor_state[motor] = state_txt or self.motor_state.get(motor, "IDLE")
                self.motor_position[motor] = int(match.group(2))
                tgt_txt = match.group(3)
                self.motor_target[motor] = int(tgt_txt) if tgt_txt is not None else None
            except Exception:
                continue

    def drain_events(self, limit=200):
        self._flush_sim()
        with self.lock:
            amount = _clamp(int(limit), 1, 2000)
            out = list(self.events)[-amount:]
            self.events.clear()
            return out

    def _record_serial_exchange_locked(self, direction, line, kind, code=None, meta=None):
        event = {
            "ts": datetime.utcnow().replace(microsecond=0).isoformat(),
            "direction": str(direction or ""),
            "kind": str(kind or ""),
            "line": str(line or ""),
            "code": code if code in (0, 1, 2) else None,
            "simulate": bool(self.simulate),
            "action": (meta or {}).get("action"),
            "motor": (meta or {}).get("motor"),
            "cmd": (meta or {}).get("cmd"),
            "position": (meta or {}).get("position"),
            "error": (meta or {}).get("error"),
        }
        self.serial_exchange.append(event)
        if self.serial_log_file is not None:
            try:
                with self.serial_log_file.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(event, ensure_ascii=False) + "\n")
            except Exception:
                pass

    def serial_snapshot(self, limit=200):
        self._flush_sim()
        with self.lock:
            amount = _clamp(int(limit), 1, 5000)
            return list(self.serial_exchange)[-amount:]

    def send_interactive(self, command):
        text = str(command or "").strip()
        if not text:
            return False
        parts = [part.strip() for part in text.split(",") if str(part).strip() != ""]
        action = None
        motor = None
        if parts:
            try:
                action = int(parts[0])
            except Exception:
                action = None
        if len(parts) > 1 and parts[1].isalpha():
            motor = parts[1]
        return self.send_raw(text, action=action, motor=motor)

    def status(self):
        self._flush_sim()
        with self.lock:
            return {
                "simulate": self.simulate,
                "port": self.port,
                "baudrate": self.baudrate,
                "last_error": self.last_error,
                "last_tx_at": self.last_tx_at,
                "last_rx_at": self.last_rx_at,
                "last_response": self.last_response,
                "last_debug_line": self.last_debug_line,
                "system_state": self.system_state,
                "calibration_done": bool(self.calibration_done),
                "motor_state": dict(self.motor_state),
                "motor_position": dict(self.motor_position),
                "motor_target": dict(self.motor_target),
                "sim_steps": dict(self.sim_steps),
                "last_commands": list(self.last_commands)[-20:],
                "responses": list(self.responses)[-20:],
                "serial_log_size": len(self.serial_exchange),
            }

    def close(self):
        self.shutdown.set()
        if self.rx_thread is not None and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.4)
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass


class MovementState(Enum):
    IDLE = "IDLE"
    MISSION_TO_TARGET = "MISSION_TO_TARGET"
    TRACKING_WITH_CAMERA = "TRACKING_WITH_CAMERA"
    MANUAL_CONTROL = "MANUAL_CONTROL"


class AstronomicOperator:
    def __init__(
        self,
        base_catalog,
        latitude=0.0,
        longitude=0.0,
        altitude_m=0.0,
        data_root=".",
        simulate_arduino=True,
        arduino_port="/dev/ttyUSB0",
        arduino_baudrate=115200,
        serial_log_file=None,
        serial_log_max=4000,
        ra_steps_per_degree=(2442.96 * 18.0 * 23.0) / 360.0,
        dec_steps_per_degree=(2442.96 * 18.0) / 360.0,
        focus_steps_total=10116.0,
        fov_x_deg=1.2,
        fov_y_deg=0.9,
    ):
        self.base_catalog = copy.deepcopy(base_catalog or {})
        self.catalog_tabs = list(self.base_catalog.keys())
        self.latitude = float(latitude or 0.0)
        self.longitude = float(longitude or 0.0)
        self.altitude_m = float(altitude_m or 0.0)
        self.ra_steps_per_degree = float(ra_steps_per_degree)
        self.dec_steps_per_degree = float(dec_steps_per_degree)
        self.focus_steps_total = float(focus_steps_total)
        self.fov_x_deg = float(fov_x_deg)
        self.fov_y_deg = float(fov_y_deg)

        self._location_received = bool(
            abs(float(self.latitude)) > 1e-9 or abs(float(self.longitude)) > 1e-9
        )
        self._time_received = True
        self._time_base_ms = int(time.time() * 1000)
        self._time_sync_mono = time.monotonic()
        self._last_shared_time_ms = None
        self._last_shared_gps = None

        self.skyfield_ready = False
        self.time_scale = None
        self.solar_system = None
        self.earth = None
        self.observer = None
        self.stars = None
        self.celestial_by_key = {}

        self.focus_mode = "auto"
        self.focus_manual_value = 50
        self.focus_manual_ticks = 0.0
        self.focus_best_step = 0.0
        self.focus_current_step = 0.0
        self.focus_target_step = 0.0
        self.focus_deadband_ticks = 10.0
        self.focus_last_command_step = None
        self.focus_last_command_mode = ""

        self.camera_angle_deg = 45.0
        self.tracking_dx = 0.0
        self.tracking_dy = 0.0
        self.tracking_frame_width = 1
        self.tracking_frame_height = 1
        self.tracking_ra_offset_deg = 0.0
        self.tracking_dec_offset_deg = 0.0

        # Manual/mission/tracking movement behavior.
        self.state_of_telescope = MovementState.IDLE
        self.active_target_id = "STANDBY"
        self.joystick_deadzone = 0.02
        self.manual_trim_limit_deg = 45.0
        self.joystick_ra_offset_deg = 0.0
        self.joystick_dec_offset_deg = 0.0
        self.sidereal_ra_deg_s = 360.0 / 86164.0905
        self.manual_speed_max_ra_deg_s = 5.0 * self.sidereal_ra_deg_s
        self.manual_speed_max_dec_deg_s = self.manual_speed_max_ra_deg_s / 24.0

        self.last_ra_step = 0
        self.last_dec_step = 0
        self.last_target_id = "STANDBY"
        self.last_ha_deg = 0.0
        self.last_dec_deg = 0.0
        self.last_tick_at = 0.0
        self.last_track_send_s = 0.0
        self.last_focus_send_s = 0.0
        self.last_send_heartbeat_s = 0.0
        self.last_sent_ra_step = 0
        self.last_sent_dec_step = 0
        self.mount_deadband_steps = 1
        self._locked_target_id = None
        self._locked_target_hadec = None
        self.reference_target_id = "11767"  # Polaris anchor for mount frame.
        self.reference_mount_ha_deg = 0.0
        self.reference_mount_dec_deg = 0.0
        self.reference_frame_ready = False
        self.reference_captured_ms = None
        self.max_ra_abs_error_steps = 80000
        self.max_dec_abs_error_steps = 2500

        self.hardware_ready = False
        self.focus_ready = False
        self.tracking_enabled = False
        self.message = "Waiting for first GPS and time sync."
        self.last_error = ""
        self.lock = Lock()
        self.ui_exchange = deque(maxlen=1000)

        # Catalog cache: update rows only when altitude/visibility/events changed.
        self.catalog_min_refresh_s = 2.0
        self.catalog_altitude_epsilon_deg = 0.02
        self.catalog_event_refresh_s = 600.0
        self.catalog_default_delay_s = 20.0
        self._catalog_row_cache = {}
        self._catalog_last_payload = copy.deepcopy(self.base_catalog)
        self._catalog_last_hash = ""
        self._catalog_last_compute_s = 0.0

        self.motor_link = MotorLink(
            simulate=simulate_arduino,
            port=arduino_port,
            baudrate=arduino_baudrate,
            log_file=serial_log_file,
            log_max=serial_log_max,
        )

        self._init_skyfield(Path(data_root))
        self._build_celestial_catalog()
        self._update_observer()
        self._refresh_message()

    @property
    def ready(self):
        return bool(self._location_received and self._time_received)

    def _init_skyfield(self, data_root):
        if Loader is None:
            self.message = "Skyfield unavailable."
            self.last_error = "Skyfield not installed."
            return
        for root in (data_root / "skyfield-data", Path.home() / ".skyfield"):
            try:
                root.mkdir(parents=True, exist_ok=True)
                loader = Loader(str(root), expire=False)
                self.time_scale = loader.timescale()
                self.solar_system = loader("de421.bsp")
                self.earth = self.solar_system["earth"]
                try:
                    with loader.open(hipparcos.URL) as handle:
                        self.stars = hipparcos.load_dataframe(handle)
                except Exception:
                    self.stars = None
                self.skyfield_ready = True
                return
            except Exception as exc:
                self.last_error = str(exc)

    def _build_celestial_catalog(self):
        self.celestial_by_key = {}
        if Star is None:
            return
        for tab_name, entries in self.base_catalog.items():
            for item in entries:
                oid = item.get("id")
                name = str(item.get("name", oid or "")).upper()
                obj = None
                if tab_name == "tab-solar" and self.solar_system is not None:
                    try:
                        obj = self.solar_system[str(oid)]
                    except Exception:
                        try:
                            obj = self.solar_system[str(oid) + " barycenter"]
                        except Exception:
                            obj = None
                elif tab_name == "tab-stars" and self.stars is not None:
                    try:
                        obj = Star.from_dataframe(self.stars.loc[int(str(oid))])
                    except Exception:
                        obj = None
                elif tab_name == "tab-deep":
                    try:
                        obj = Star(
                            ra_hours=float(item.get("ra_hours")),
                            dec_degrees=float(item.get("dec_degrees")),
                        )
                    except Exception:
                        obj = None
                if obj is not None:
                    if oid is not None:
                        self.celestial_by_key[str(oid).upper()] = obj
                    self.celestial_by_key[name] = obj

    def _resolve_target(self, target_id):
        return self.celestial_by_key.get(str(target_id or "").upper())

    def _catalog_key(self, tab_name, row):
        ident = row.get("id")
        if ident is None:
            ident = row.get("name", "")
        return f"{tab_name}:{str(ident).upper()}"

    def _update_observer(self):
        if self.earth is None or wgs84 is None:
            self.observer = None
        else:
            self.observer = self.earth + wgs84.latlon(
                self.latitude, self.longitude, elevation_m=self.altitude_m
            )

    def _refresh_message(self):
        needs = []
        if not self._location_received:
            needs.append("GPS")
        if not self._time_received:
            needs.append("time/date")
        self.message = (
            "Astronomic operator ready."
            if not needs
            else ("Waiting for " + " and ".join(needs) + ".")
        )

    def _set_location_locked(self, latitude, longitude, altitude_m=0.0):
        self.latitude = float(latitude or 0.0)
        self.longitude = float(longitude or 0.0)
        self.altitude_m = float(altitude_m or 0.0)
        self._location_received = True
        self._update_observer()
        self._refresh_message()

    def _set_time_locked(self, timestamp_ms=None, datetime_value=None):
        parsed = None
        if timestamp_ms is not None:
            try:
                parsed = int(float(timestamp_ms))
            except Exception:
                parsed = None
        if parsed is None:
            parsed = _parse_datetime_to_ms(datetime_value)
        self._time_base_ms = int(parsed if parsed is not None else (time.time() * 1000))
        self._time_sync_mono = time.monotonic()
        self._time_received = True
        self._refresh_message()

    def _current_timestamp_ms(self):
        if self._time_base_ms is None or self._time_sync_mono is None:
            return int(time.time() * 1000)
        elapsed = int((time.monotonic() - self._time_sync_mono) * 1000)
        return int(self._time_base_ms + elapsed)

    def _time_from_ms(self, timestamp_ms):
        if self.time_scale is None:
            return None
        dt_utc = None
        try:
            dt_utc = datetime.fromtimestamp(float(timestamp_ms) / 1000.0, tz=timezone.utc)
        except Exception:
            dt_utc = None
        if dt_utc is not None:
            try:
                return self.time_scale.from_datetime(dt_utc)
            except Exception:
                pass
        try:
            return self.time_scale.utc(datetime.utcfromtimestamp(float(timestamp_ms) / 1000.0))
        except Exception:
            return None

    def set_location(self, latitude, longitude, altitude_m=0.0):
        with self.lock:
            self._set_location_locked(latitude, longitude, altitude_m)

    def updateLocation(self, lat, lon):
        self.set_location(lat, lon, self.altitude_m)

    def set_time(self, timestamp_ms=None, datetime_value=None):
        with self.lock:
            self._set_time_locked(timestamp_ms=timestamp_ms, datetime_value=datetime_value)

    def set_command_gates(self, hardware_ready, focus_ready, tracking_enabled):
        with self.lock:
            prev_hardware_ready = bool(self.hardware_ready)
            prev_focus_ready = bool(self.focus_ready)
            self.hardware_ready = bool(hardware_ready)
            self.focus_ready = bool(focus_ready)
            self.tracking_enabled = bool(tracking_enabled)
            if not prev_hardware_ready and self.hardware_ready:
                self._capture_reference_frame_locked(self._current_timestamp_ms())
            elif prev_hardware_ready and not self.hardware_ready:
                self.reference_frame_ready = False
                self.reference_captured_ms = None
            if (not prev_hardware_ready and self.hardware_ready) or (
                not prev_focus_ready and self.focus_ready
            ):
                self.focus_last_command_step = None

    def on_ui_packet(self, direction, packet, meta=None):
        with self.lock:
            self.ui_exchange.append(
                {
                    "ts": datetime.utcnow().replace(microsecond=0).isoformat(),
                    "direction": str(direction or "unknown"),
                    "type": str((packet or {}).get("type", "UNKNOWN")),
                    "meta": copy.deepcopy(meta or {}),
                    "packet": copy.deepcopy(packet or {}),
                }
            )

    def set_tracking_error(self, dx, dy, frame_width, frame_height):
        with self.lock:
            self.tracking_dx = float(dx or 0.0)
            self.tracking_dy = float(dy or 0.0)
            self.tracking_frame_width = max(1, int(frame_width or 1))
            self.tracking_frame_height = max(1, int(frame_height or 1))

    def set_focus_info(self, mode, manual_value, best_step, current_step, manual_ticks=None):
        with self.lock:
            prev_mode = str(self.focus_mode)
            if mode in {"auto", "manual"}:
                self.focus_mode = mode
            self.focus_manual_value = _clamp(int(manual_value or 0), 0, 100)
            if manual_ticks is None:
                self.focus_manual_ticks = (self.focus_manual_value / 100.0) * self.focus_steps_total
            else:
                self.focus_manual_ticks = _clamp(float(manual_ticks), 0.0, self.focus_steps_total)
            self.focus_best_step = float(best_step or 0.0)
            self.focus_current_step = float(current_step or 0.0)
            self.focus_target_step = (
                self.focus_best_step if self.focus_mode == "auto" else self.focus_manual_ticks
            )
            if self.focus_mode != prev_mode:
                self.focus_last_command_step = None
                self.focus_last_command_mode = ""

    @staticmethod
    def new_ha_dec(ha, dec):
        new_ha = (ha % 360.0 - 90.0) % 180.0 - 90.0
        if ha == -90:
            new_ha = 90.0
        new_dec = dec
        if abs(ha) >= 90.0:
            new_dec = (180.0 - dec % 360.0) % 360.0
        return new_ha, new_dec
    
    
    def rotate_x_y_by_angle_deg(self, x, y, angle_deg):
        a = math.radians(float(angle_deg))
        new_x = (x * math.cos(a)) - (y * math.sin(a))
        new_y = (x * math.sin(a)) + (y * math.cos(a))
        return float(new_x), float(new_y)
    
    def x_y_to_ha_dec(self, x_deg, y_deg, camera_angle_deg, dec_position_deg):
        delta_x, delta_y = self.rotate_x_y_by_angle_deg(x_deg, y_deg, camera_angle_deg)
        # For small DEC angles, the HA correction becomes unstable (division by sin(dec) ~ 0), so we can approximate:
        if abs(dec_position_deg) > 5:
            ha_offset = delta_y / math.sin(math.radians(dec_position_deg))
            dec_offset = (
                delta_y * math.cos(math.radians(dec_position_deg))
                / math.sin(math.radians(dec_position_deg))
                - delta_x
            )
        else:
            ha_offset = 0.0
            dec_offset = float(delta_x)

        return float(ha_offset), float(dec_offset)

    def _target_hadec(self, target_id, timestamp_ms):
        obj = self._resolve_target(target_id)
        if obj is None or self.observer is None or self.time_scale is None:
            return None
        t_now = self._time_from_ms(timestamp_ms)
        if t_now is None:
            return None
        try:
            ha, dec, _ = self.observer.at(t_now).observe(obj).apparent().hadec()
            return float(ha.degrees), float(dec.degrees)
        except Exception:
            return None

    def _capture_reference_frame_locked(self, timestamp_ms=None):
        ts_ms = int(timestamp_ms if timestamp_ms is not None else self._current_timestamp_ms())
        polaris = self._target_hadec(self.reference_target_id, ts_ms)
        if polaris is None:
            self.reference_frame_ready = False
            return False
        mount_ha, mount_dec = self.new_ha_dec(*polaris)
        self.reference_mount_ha_deg = float(mount_ha)
        self.reference_mount_dec_deg = float(mount_dec)
        self.reference_frame_ready = True
        self.reference_captured_ms = ts_ms
        return True

    def _apply_reference_offset_locked(self, mount_ha_deg, mount_dec_deg):
        if not self.reference_frame_ready:
            return float(mount_ha_deg), float(mount_dec_deg)
        return (
            float(mount_ha_deg) - float(self.reference_mount_ha_deg),
            float(mount_dec_deg) - float(self.reference_mount_dec_deg),
        )

    @staticmethod
    def _limit_target_error_against_position(target_step, current_step, max_abs_error):
        try:
            tgt = int(round(float(target_step)))
        except Exception:
            return 0
        if current_step is None:
            return tgt
        try:
            cur = int(round(float(current_step)))
        except Exception:
            return tgt
        try:
            max_err = int(max(1, abs(int(max_abs_error))))
        except Exception:
            max_err = 1
        delta = tgt - cur
        if abs(delta) <= max_err:
            return tgt
        return int(cur + (max_err if delta > 0 else -max_err))

    def _tracking_offsets_deg_locked(self):
        w = max(1.0, float(self.tracking_frame_width))
        h = max(1.0, float(self.tracking_frame_height))
        x_deg = (float(self.tracking_dx) / w) * float(self.fov_x_deg)
        y_deg = (-float(self.tracking_dy) / h) * float(self.fov_y_deg)

        # Rotate camera-frame correction into mount HA/DEC frame.
        a = math.radians(float(self.camera_angle_deg))
        ha_offset, dec_offset  = self.x_y_to_ha_dec(x_deg, y_deg, self.camera_angle_deg, self.last_dec_deg)
        self.tracking_ra_offset_deg = float(ha_offset)
        self.tracking_dec_offset_deg = float(dec_offset)
        return float(ha_offset), float(dec_offset)

    def _sync_time_and_location_from_shared_locked(self, shared_data):
        payload = dict(shared_data or {})
        gps = dict(payload.get("gps", {}))
        lat = gps.get("lat")
        lon = gps.get("lon")
        alt = gps.get("alt", self.altitude_m)
        try:
            new_gps = (round(float(lat), 7), round(float(lon), 7), round(float(alt or 0.0), 3))
        except Exception:
            new_gps = None
        can_accept_gps = bool(self._location_received)
        if not can_accept_gps and new_gps is not None:
            can_accept_gps = abs(new_gps[0]) > 1e-9 or abs(new_gps[1]) > 1e-9
        if new_gps is not None and can_accept_gps and new_gps != self._last_shared_gps:
            self._last_shared_gps = new_gps
            self._set_location_locked(new_gps[0], new_gps[1], new_gps[2])

        ts_ms = payload.get("timestamp")
        dt_str = payload.get("datetime")
        parsed_ts = None
        try:
            if ts_ms is not None:
                parsed_ts = int(float(ts_ms))
        except Exception:
            parsed_ts = None
        if parsed_ts is None:
            parsed_ts = _parse_datetime_to_ms(dt_str)
        if parsed_ts is not None and self._time_received:
            if self._last_shared_time_ms is None or abs(parsed_ts - self._last_shared_time_ms) >= 500:
                self._last_shared_time_ms = parsed_ts
                self._set_time_locked(timestamp_ms=parsed_ts)

    def _compute_visibility_event_fields_locked(self, cb):
        rise, set_, visible = self.rising_setting_time(cb)
        rise25, set25 = self.rising_setting_above_alt(cb, 25.0)
        rise45, set45 = self.rising_setting_above_alt(cb, 45.0)
        return {
            "rising": rise.isoformat() if rise is not None else None,
            "setting": set_.isoformat() if set_ is not None else None,
            "rising25": rise25.isoformat() if rise25 is not None else None,
            "setting25": set25.isoformat() if set25 is not None else None,
            "rising45": rise45.isoformat() if rise45 is not None else None,
            "setting45": set45.isoformat() if set45 is not None else None,
            "visible_window": bool(visible),
        }

    @staticmethod
    def _visibility_level_for_altitude(altitude_deg):
        try:
            alt = float(altitude_deg)
        except Exception:
            return "hidden"
        if alt < 0.0:
            return "hidden"
        if alt >= 45.0:
            return "high"
        if alt >= 25.0:
            return "medium"
        return "low"

    def compute_visible_catalog(self, timestamp_ms=None):
        with self.lock:
            now_s = time.monotonic()
            fallback_catalog = (
                copy.deepcopy(self._catalog_last_payload)
                if self._catalog_last_payload is not None
                else copy.deepcopy(self.base_catalog)
            )
            if not self.ready or not self.skyfield_ready or self.observer is None:
                return fallback_catalog, 5.0

            if (
                self._catalog_last_payload is not None
                and (now_s - self._catalog_last_compute_s) < float(self.catalog_min_refresh_s)
            ):
                retry_in = max(1.0, float(self.catalog_min_refresh_s) - (now_s - self._catalog_last_compute_s))
                return copy.deepcopy(self._catalog_last_payload), retry_in

            ts_ms = int(timestamp_ms if timestamp_ms is not None else self._current_timestamp_ms())
            t_now = self._time_from_ms(ts_ms)
            if t_now is None:
                try:
                    t_now = self.time_scale.now()
                except Exception:
                    t_now = None
            if t_now is None:
                return fallback_catalog, 5.0

            changed = False
            out = {tab: [] for tab in self.catalog_tabs}
            seen_keys = set()

            for tab_name, entries in self.base_catalog.items():
                for raw in entries:
                    key = self._catalog_key(tab_name, raw)
                    seen_keys.add(key)
                    obj = self._resolve_target(raw.get("id")) or self._resolve_target(raw.get("name"))
                    cache = dict(self._catalog_row_cache.get(key, {}))
                    if obj is None:
                        if cache.get("visible", False):
                            changed = True
                        self._catalog_row_cache[key] = {
                            "visible": False,
                            "row": None,
                            "altitude_deg": None,
                            "events": cache.get("events", {}),
                            "next_event_refresh_s": now_s + float(self.catalog_event_refresh_s),
                        }
                        continue

                    try:
                        alt, _, _ = self.observer.at(t_now).observe(obj).apparent().altaz()
                        altitude_deg = float(alt.degrees)
                    except Exception:
                        altitude_deg = None

                    if altitude_deg is None or altitude_deg < 0.0:
                        if cache.get("visible", False):
                            changed = True
                        self._catalog_row_cache[key] = {
                            "visible": False,
                            "row": None,
                            "altitude_deg": altitude_deg,
                            "events": cache.get("events", {}),
                            "next_event_refresh_s": cache.get(
                                "next_event_refresh_s", now_s + float(self.catalog_event_refresh_s)
                            ),
                        }
                        continue

                    rounded_alt = round(float(altitude_deg), 2)
                    events = dict(cache.get("events", {}))
                    next_event_refresh_s = float(
                        cache.get("next_event_refresh_s", now_s + float(self.catalog_event_refresh_s))
                    )
                    if not events or now_s >= next_event_refresh_s:
                        events = self._compute_visibility_event_fields_locked(obj)
                        next_event_refresh_s = now_s + float(self.catalog_event_refresh_s)
                        changed = True

                    row = copy.deepcopy(raw)
                    row["altitude_deg"] = rounded_alt
                    row["visibility_level"] = self._visibility_level_for_altitude(rounded_alt)
                    if events:
                        row.update(events)
                    out[tab_name].append(row)

                    previous_alt = cache.get("altitude_deg")
                    previous_row = cache.get("row")
                    if (
                        not cache.get("visible", False)
                        or previous_alt is None
                        or abs(float(previous_alt) - rounded_alt) >= float(self.catalog_altitude_epsilon_deg)
                        or previous_row != row
                    ):
                        changed = True

                    self._catalog_row_cache[key] = {
                        "visible": True,
                        "row": row,
                        "altitude_deg": rounded_alt,
                        "events": events,
                        "next_event_refresh_s": next_event_refresh_s,
                    }

                out[tab_name].sort(
                    key=lambda x: (
                        float(x.get("altitude_deg"))
                        if isinstance(x.get("altitude_deg"), (int, float))
                        else -999.0
                    ),
                    reverse=True,
                )

            # Prune removed rows.
            stale_keys = [key for key in self._catalog_row_cache.keys() if key not in seen_keys]
            for key in stale_keys:
                self._catalog_row_cache.pop(key, None)
                changed = True

            # Fallback safety: avoid publishing an empty catalog when compute fails silently.
            if not any(len(entries) for entries in out.values()):
                return fallback_catalog, 5.0

            payload_hash = ""
            try:
                payload_hash = json.dumps(out, sort_keys=True, ensure_ascii=True)
            except Exception:
                payload_hash = str(out)
            if payload_hash != self._catalog_last_hash:
                changed = True
                self._catalog_last_hash = payload_hash

            self._catalog_last_payload = out
            self._catalog_last_compute_s = now_s

            delay = 8.0 if changed else float(self.catalog_default_delay_s)
            return copy.deepcopy(out), delay

    def _update_focus_command_locked(self, now):
        self.focus_target_step = (
            self.focus_best_step if self.focus_mode == "auto" else self.focus_manual_ticks
        )
        if not (self.hardware_ready and self.focus_ready):
            return
        target_step = float(self.focus_target_step)
        should_send_focus = False
        if self.focus_last_command_step is None:
            should_send_focus = True
        elif abs(target_step - float(self.focus_last_command_step)) > float(self.focus_deadband_ticks):
            should_send_focus = True
        elif self.focus_last_command_mode != self.focus_mode:
            should_send_focus = True

        if should_send_focus and (now - self.last_focus_send_s) >= 0.08:
            sent = self.motor_link.send_command(1, "FOC", int(round(target_step)))
            if sent:
                self.focus_last_command_step = target_step
                self.focus_last_command_mode = str(self.focus_mode)
            self.last_focus_send_s = now

    def _classify_state_locked(self, target_selected, camera_tracking_enabled, manual_input_active):
        if not target_selected and manual_input_active:
            return MovementState.MANUAL_CONTROL
        if target_selected and camera_tracking_enabled:
            return MovementState.TRACKING_WITH_CAMERA
        if target_selected:
            return MovementState.MISSION_TO_TARGET
        return MovementState.IDLE

    def tick(self, shared_data):
        with self.lock:
            now = time.monotonic()
            dt = 0.04 if self.last_tick_at <= 0 else _clamp(now - self.last_tick_at, 0.001, 0.25)
            self.last_tick_at = now

            self._sync_time_and_location_from_shared_locked(shared_data)
            if not self.ready:
                return

            self._update_focus_command_locked(now)

            mission = dict((shared_data or {}).get("mission", {}))
            motor_flags = dict((shared_data or {}).get("motor", {}))
            joy = dict((shared_data or {}).get("joystick", {}))
            jx = float(joy.get("x", 0.0) or 0.0)
            jy = float(joy.get("y", 0.0) or 0.0)
            manual_input_active = (abs(jx) >= self.joystick_deadzone) or (abs(jy) >= self.joystick_deadzone)

            target_id = str(mission.get("target", "STANDBY"))
            target_selected = target_id.upper() != "STANDBY"
            if target_id.upper() != self.active_target_id:
                self.active_target_id = target_id.upper()
                self.joystick_ra_offset_deg = 0.0
                self.joystick_dec_offset_deg = 0.0
                self._locked_target_id = None
                self._locked_target_hadec = None

            camera_angle = (
                motor_flags.get("camera_angle_deg")
                if "camera_angle_deg" in motor_flags
                else dict((shared_data or {}).get("vision_status", {})).get("camera_angle_deg")
            )
            try:
                if camera_angle is not None:
                    self.camera_angle_deg = float(camera_angle)
            except Exception:
                pass

            tracking_with_camera = bool(motor_flags.get("tracking_with_camera", True))
            compensate_earth_rotation = bool(motor_flags.get("compensate_earth_rotation", True))
            camera_tracking_enabled = bool(tracking_with_camera and self.tracking_enabled and target_selected)

            self.state_of_telescope = self._classify_state_locked(
                target_selected=target_selected,
                camera_tracking_enabled=camera_tracking_enabled,
                manual_input_active=manual_input_active,
            )

            ts_ms = int((shared_data or {}).get("timestamp") or self._current_timestamp_ms())
            desired_ha_deg = float(self.last_ha_deg)
            desired_dec_deg = float(self.last_dec_deg)

            if target_selected:
                hadec = None
                if compensate_earth_rotation:
                    hadec = self._target_hadec(target_id, ts_ms)
                    if hadec is not None:
                        self._locked_target_id = str(target_id)
                        self._locked_target_hadec = (float(hadec[0]), float(hadec[1]))
                else:
                    if self._locked_target_id != str(target_id) or self._locked_target_hadec is None:
                        locked = self._target_hadec(target_id, ts_ms)
                        if locked is not None:
                            self._locked_target_id = str(target_id)
                            self._locked_target_hadec = (float(locked[0]), float(locked[1]))
                    hadec = self._locked_target_hadec
                print(f"hadec: {hadec}", flush=True)
                if hadec is not None:
                    mount_ha_deg, mount_dec_deg = self.new_ha_dec(*hadec)
                    base_ha_deg, base_dec_deg = mount_ha_deg, mount_dec_deg#self._apply_reference_offset_locked(
                     #   mount_ha_deg, mount_dec_deg
                    #)
                    print(f"base_ha_deg: {base_ha_deg}, base_dec_deg {base_dec_deg}", flush = True)
                    if manual_input_active:
                        joystick_ra_add, joystick_dec_add = self.x_y_to_ha_dec(
                            x_deg=jx * self.manual_speed_max_ra_deg_s * dt,
                            y_deg=jy * self.manual_speed_max_dec_deg_s * dt,
                            camera_angle_deg=self.camera_angle_deg,
                            dec_position_deg=base_dec_deg,
                        )

                        self.joystick_ra_offset_deg = _clamp(
                            self.joystick_ra_offset_deg + joystick_ra_add,
                            -self.manual_trim_limit_deg,
                            self.manual_trim_limit_deg,
                        )
                        self.joystick_dec_offset_deg = _clamp(
                            self.joystick_dec_offset_deg + joystick_dec_add,
                            -self.manual_trim_limit_deg,
                            self.manual_trim_limit_deg,
                        )
                    desired_ha_deg = float(base_ha_deg) + float(self.joystick_ra_offset_deg)
                    desired_dec_deg = float(base_dec_deg) + float(self.joystick_dec_offset_deg)
                    print(f"desired_ha_deg: {desired_ha_deg}, desired_dec_deg: {desired_dec_deg}", flush = True)
                    if camera_tracking_enabled:
                        ra_off, dec_off = self._tracking_offsets_deg_locked()
                        desired_ha_deg += ra_off
                        desired_dec_deg += dec_off
                    else:
                        self.tracking_ra_offset_deg = 0.0
                        self.tracking_dec_offset_deg = 0.0
            else:
                # Manual and sidereal behavior without active mission target.
                self.tracking_ra_offset_deg = 0.0
                self.tracking_dec_offset_deg = 0.0
                self.joystick_ra_offset_deg = 0.0
                self.joystick_dec_offset_deg = 0.0
                joystick_ra_add, joystick_dec_add = self.x_y_to_ha_dec(
                            x_deg=jx * self.manual_speed_max_ra_deg_s * dt,
                            y_deg=jy * self.manual_speed_max_dec_deg_s * dt,
                            camera_angle_deg=self.camera_angle_deg,
                            dec_position_deg=base_dec_deg,
                        )
                if manual_input_active:
                    desired_ha_deg += joystick_ra_add
                    desired_dec_deg += joystick_dec_add
                if compensate_earth_rotation:
                    desired_ha_deg += self.sidereal_ra_deg_s * dt

            desired_ha_deg = _clamp(desired_ha_deg, -90.0, 90.0)
            desired_dec_deg = _clamp(desired_dec_deg, -180.0, 180.0)
            print(f"desired_ha_deg2: {desired_ha_deg}, desired_dec_deg2: {desired_dec_deg}", flush = True)

            self.last_target_id = target_id
            self.last_ha_deg = float(desired_ha_deg)
            self.last_dec_deg = float(desired_dec_deg)
            self.last_ra_step = int(round(float(desired_ha_deg) * self.ra_steps_per_degree))
            self.last_dec_step = int(round(float(desired_dec_deg) * self.dec_steps_per_degree))
            print(f"self.last_ra_step: {self.last_ra_step}, last_dec_step{self.last_dec_step}", flush = True)
            # Mount movement does not depend on focus-ready; only hardware calibration gate.
            if not self.hardware_ready:
                return

            link_status = self.motor_link.status()
            motor_state = dict(link_status.get("motor_state", {}))
            motor_position = dict(link_status.get("motor_position", {}))
            ra_feedback_step = motor_position.get("RA")
            dec_feedback_step = motor_position.get("DEC")
            if ra_feedback_step is None:
                ra_feedback_step = self.last_sent_ra_step
            if dec_feedback_step is None:
                dec_feedback_step = self.last_sent_dec_step
            if (
                str(link_status.get("system_state", "")).upper() == "ERROR"
                or str(motor_state.get("RA", "")).upper() == "ERROR"
                or str(motor_state.get("DEC", "")).upper() == "ERROR"
            ):
                self.last_error = (
                    "Mount controller is in ERROR state. "
                    "Movement commands are paused until recalibration."
                )
                return

            cmd_ra_step = self._limit_target_error_against_position(
                self.last_ra_step,
                ra_feedback_step,
                self.max_ra_abs_error_steps,
            )
            cmd_dec_step = self._limit_target_error_against_position(
                self.last_dec_step,
                dec_feedback_step,
                self.max_dec_abs_error_steps,
            )

            manual_drive_active = bool(
                self.state_of_telescope == MovementState.MANUAL_CONTROL
                or (target_selected and manual_input_active)
            )
            step_changed = (
                self.last_sent_ra_step is None
                or self.last_sent_dec_step is None
                or abs(int(cmd_ra_step) - int(self.last_sent_ra_step)) >= int(self.mount_deadband_steps)
                or abs(int(cmd_dec_step) - int(self.last_sent_dec_step)) >= int(self.mount_deadband_steps)
                or manual_drive_active
            )
            heartbeat_due = (now - self.last_send_heartbeat_s) >= 2.0
            slew_throttled = bool(
                int(cmd_ra_step) != int(self.last_ra_step)
                or int(cmd_dec_step) != int(self.last_dec_step)
            )
            send_interval = 0.4 if slew_throttled else 0.02
            if (step_changed or heartbeat_due) and (now - self.last_track_send_s) >= send_interval:
                print(f"Latitude: {self.latitude}, Longitude: {self.longitude}", flush=True)
                print(f"cmd_ra_step:{cmd_ra_step}, cmd_dec_step: {cmd_dec_step}", flush = True)
                sent_ra = self.motor_link.send_command(1, "RA", cmd_ra_step)
                time.sleep(0.04)
                sent_dec = self.motor_link.send_command(1, "DEC", cmd_dec_step)
                if sent_ra:
                    self.last_sent_ra_step = int(cmd_ra_step)
                if sent_dec:
                    self.last_sent_dec_step = int(cmd_dec_step)
                self.last_track_send_s = now
                self.last_send_heartbeat_s = now

    def print_HaDec(self, cb, name=""):
        if cb is None:
            return None, None
        t_now = self._time_from_ms(self._current_timestamp_ms())
        if t_now is None or self.observer is None:
            return None, None
        try:
            ha, dec, _ = self.observer.at(t_now).observe(cb).apparent().hadec()
            return self.new_ha_dec(float(ha.degrees), float(dec.degrees))
        except Exception:
            return None, None

    def rising_setting_time(self, cb):
        if cb is None or almanac is None or self.time_scale is None or self.observer is None:
            return None, None, False
        try:
            t0 = self.time_scale.now()
            t1 = self.time_scale.utc(t0.utc_datetime() + timedelta(days=2))
            rising, _ = almanac.find_risings(self.observer, cb, t0, t1)
            setting, _ = almanac.find_settings(self.observer, cb, t0, t1)
            if len(rising) == 0 or len(setting) == 0:
                return None, None, False
            rise = rising[0].utc_datetime()
            set_ = setting[0].utc_datetime()
            return rise, set_, bool(rise > set_)
        except Exception:
            return None, None, False

    def rising_setting_above_alt(self, cb, altitude_deg):
        if cb is None or almanac is None or self.time_scale is None or self.observer is None:
            return None, None

        def predicate(t):
            alt, _, _ = self.observer.at(t).observe(cb).apparent().altaz()
            return alt.degrees > float(altitude_deg)

        predicate.step_days = 0.01
        try:
            t0 = self.time_scale.now()
            t1 = self.time_scale.utc(t0.utc_datetime() + timedelta(days=3))
            times, values = almanac.find_discrete(t0, t1, predicate)
            rise = None
            set_ = None
            for i in range(1, len(values)):
                if not values[i - 1] and values[i]:
                    rise = times[i].utc_datetime()
                    break
            for i in range(1, len(values)):
                if values[i - 1] and not values[i]:
                    set_ = times[i].utc_datetime()
                    break
            return rise, set_
        except Exception:
            return None, None

    def print_visible_catalog(self):
        catalog, _ = self.compute_visible_catalog(timestamp_ms=self._current_timestamp_ms())
        return catalog

    def status(self):
        with self.lock:
            return {
                "ready": self.ready,
                "time_synced": bool(self._time_received),
                "has_location": bool(self._location_received),
                "skyfield_ready": bool(self.skyfield_ready),
                "message": self.message,
                "last_error": self.last_error,
                "movement_state": self.state_of_telescope.value,
                "target": {
                    "id": self.last_target_id,
                    "ha_deg": round(float(self.last_ha_deg), 6),
                    "dec_deg": round(float(self.last_dec_deg), 6),
                    "ra_step": int(self.last_ra_step),
                    "dec_step": int(self.last_dec_step),
                },
                "focus": {
                    "mode": self.focus_mode,
                    "manual_value": int(self.focus_manual_value),
                    "manual_ticks": round(float(self.focus_manual_ticks), 2),
                    "best_step": round(float(self.focus_best_step), 2),
                    "current_step": round(float(self.focus_current_step), 2),
                    "target_step": round(float(self.focus_target_step), 2),
                    "range_ticks": round(float(self.focus_steps_total), 2),
                },
                "tracking": {
                    "dx": round(float(self.tracking_dx), 4),
                    "dy": round(float(self.tracking_dy), 4),
                    "frame_width": int(self.tracking_frame_width),
                    "frame_height": int(self.tracking_frame_height),
                    "camera_angle_deg": round(float(self.camera_angle_deg), 4),
                    "ra_offset_deg": round(float(self.tracking_ra_offset_deg), 6),
                    "dec_offset_deg": round(float(self.tracking_dec_offset_deg), 6),
                    "joy_ra_offset_deg": round(float(self.joystick_ra_offset_deg), 6),
                    "joy_dec_offset_deg": round(float(self.joystick_dec_offset_deg), 6),
                },
                "reference_frame": {
                    "target_id": str(self.reference_target_id),
                    "ready": bool(self.reference_frame_ready),
                    "captured_ms": self.reference_captured_ms,
                    "ha_deg": round(float(self.reference_mount_ha_deg), 6),
                    "dec_deg": round(float(self.reference_mount_dec_deg), 6),
                },
                "gates": {
                    "hardware_ready": bool(self.hardware_ready),
                    "focus_ready": bool(self.focus_ready),
                    "tracking_enabled": bool(self.tracking_enabled),
                },
                "catalog_cache": {
                    "rows": len(self._catalog_row_cache),
                    "last_compute_s": float(self._catalog_last_compute_s),
                },
                "motor_link": self.motor_link.status(),
                "ui_exchange_size": len(self.ui_exchange),
                "last_tick_at": self.last_tick_at,
            }

    def close(self):
        self.motor_link.close()
