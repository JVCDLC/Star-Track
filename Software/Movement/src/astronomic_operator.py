import copy
import time
from collections import deque
from datetime import datetime, timedelta
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

    def __init__(self, simulate=True, port="/dev/ttyUSB0", baudrate=115200):
        self.simulate = bool(simulate)
        self.port = port
        self.baudrate = int(baudrate)
        self.serial_conn = None

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

        self.pending_tx = deque(maxlen=200)
        self.last_commands = deque(maxlen=300)
        self.responses = deque(maxlen=300)
        self.events = deque(maxlen=300)
        self.sim_pending_done = deque(maxlen=100)
        self.sim_steps = {"RA": 0.0, "DEC": 0.0, "FOC": 0.0}

        if not self.simulate and serial is not None:
            try:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.05, write_timeout=0.2)
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

            if not self.simulate:
                try:
                    self.serial_conn.write((str(command) + "\n").encode("utf-8"))
                    return True
                except Exception as exc:
                    self.last_error = str(exc)
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
                line = self.serial_conn.readline().decode("utf-8", errors="replace").strip()
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                time.sleep(0.05)
                continue
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
            try:
                code = int(str(line).strip())
            except Exception:
                code = None

            if code in (0, 1, 2):
                meta = meta_override or (self.pending_tx.popleft() if self.pending_tx else None)
                ev = {
                    "ts": now,
                    "kind": "ack",
                    "code": code,
                    "line": str(line),
                    "cmd": (meta or {}).get("cmd"),
                    "action": (meta or {}).get("action"),
                    "motor": (meta or {}).get("motor"),
                }
                self.responses.append(ev)
                self.events.append(ev)
                self._apply_code(code, meta)
            else:
                self.last_debug_line = str(line)
                self.events.append({"ts": now, "kind": "debug", "line": str(line)})

    def _flush_sim(self):
        if not self.simulate:
            return
        now = time.time()
        while self.sim_pending_done and self.sim_pending_done[0][0] <= now:
            _, meta = self.sim_pending_done.popleft()
            self._on_response_line("2", meta_override=meta)

    def drain_events(self, limit=200):
        self._flush_sim()
        with self.lock:
            amount = _clamp(int(limit), 1, 2000)
            out = list(self.events)[-amount:]
            self.events.clear()
            return out

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
                "sim_steps": dict(self.sim_steps),
                "last_commands": list(self.last_commands)[-20:],
                "responses": list(self.responses)[-20:],
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
        ra_steps_per_degree=40.0,
        dec_steps_per_degree=40.0,
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

        self._location_received = False
        self._time_received = False
        self._time_base_ms = None
        self._time_sync_mono = None

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

        self.tracking_ra_offset_deg = 0.0
        self.tracking_dec_offset_deg = 0.0
        self.joystick_ra_offset_deg = 0.0
        self.joystick_dec_offset_deg = 0.0
        self.last_ra_step = 0
        self.last_dec_step = 0
        self.last_target_id = "STANDBY"
        self.last_ha_deg = 0.0
        self.last_dec_deg = 0.0
        self.last_tick_at = 0.0
        self.last_track_send_s = 0.0
        self.last_focus_send_s = 0.0

        self.hardware_ready = False
        self.focus_ready = False
        self.tracking_enabled = False
        self.message = "Waiting for first GPS and time sync."
        self.last_error = ""
        self.lock = Lock()
        self.ui_exchange = deque(maxlen=1000)

        self.motor_link = MotorLink(
            simulate=simulate_arduino,
            port=arduino_port,
            baudrate=arduino_baudrate,
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
                        obj = Star(ra_hours=float(item.get("ra_hours")), dec_degrees=float(item.get("dec_degrees")))
                    except Exception:
                        obj = None
                if obj is not None:
                    if oid is not None:
                        self.celestial_by_key[str(oid).upper()] = obj
                    self.celestial_by_key[name] = obj

    def _resolve_target(self, target_id):
        return self.celestial_by_key.get(str(target_id or "").upper())

    def _update_observer(self):
        if self.earth is None or wgs84 is None:
            self.observer = None
        else:
            self.observer = self.earth + wgs84.latlon(self.latitude, self.longitude, elevation_m=self.altitude_m)

    def _refresh_message(self):
        needs = []
        if not self._location_received:
            needs.append("GPS")
        if not self._time_received:
            needs.append("time/date")
        self.message = "Astronomic operator ready." if not needs else ("Waiting for " + " and ".join(needs) + ".")

    def _current_timestamp_ms(self):
        if self._time_base_ms is None or self._time_sync_mono is None:
            return int(time.time() * 1000)
        elapsed = int((time.monotonic() - self._time_sync_mono) * 1000)
        return int(self._time_base_ms + elapsed)

    def _time_from_ms(self, timestamp_ms):
        try:
            return self.time_scale.utc(datetime.utcfromtimestamp(timestamp_ms / 1000.0))
        except Exception:
            return None

    def set_location(self, latitude, longitude, altitude_m=0.0):
        with self.lock:
            self.latitude = float(latitude or 0.0)
            self.longitude = float(longitude or 0.0)
            self.altitude_m = float(altitude_m or 0.0)
            self._location_received = True
            self._update_observer()
            self._refresh_message()

    def updateLocation(self, lat, lon):
        self.set_location(lat, lon, self.altitude_m)

    def set_time(self, timestamp_ms=None, datetime_value=None):
        with self.lock:
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

    def set_command_gates(self, hardware_ready, focus_ready, tracking_enabled):
        with self.lock:
            self.hardware_ready = bool(hardware_ready)
            self.focus_ready = bool(focus_ready)
            self.tracking_enabled = bool(tracking_enabled)

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
            w = max(1, int(frame_width or 1))
            h = max(1, int(frame_height or 1))
            self.tracking_ra_offset_deg = (float(dx or 0.0) / float(w)) * self.fov_x_deg
            self.tracking_dec_offset_deg = (-float(dy or 0.0) / float(h)) * self.fov_y_deg

    def set_focus_info(self, mode, manual_value, best_step, current_step, manual_ticks=None):
        with self.lock:
            if mode in {"auto", "manual"}:
                self.focus_mode = mode
            self.focus_manual_value = _clamp(int(manual_value or 0), 0, 100)
            if manual_ticks is None:
                self.focus_manual_ticks = (self.focus_manual_value / 100.0) * self.focus_steps_total
            else:
                self.focus_manual_ticks = _clamp(float(manual_ticks), 0.0, self.focus_steps_total)
            self.focus_best_step = float(best_step or 0.0)
            self.focus_current_step = float(current_step or 0.0)
            self.focus_target_step = self.focus_best_step if self.focus_mode == "auto" else self.focus_manual_ticks

    @staticmethod
    def new_ha_dec(ha, dec):
        new_ha = (ha % 360.0 - 90.0) % 180.0 - 90.0
        if ha == -90:
            new_ha = 90.0
        new_dec = dec
        if abs(ha) >= 90.0:
            new_dec = (180.0 - dec % 360.0) % 360.0
        return new_ha, new_dec

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

    def compute_visible_catalog(self, timestamp_ms=None):
        with self.lock:
            if not self.ready or not self.skyfield_ready or self.observer is None:
                return {tab: [] for tab in self.catalog_tabs}, 15.0
            ts_ms = int(timestamp_ms if timestamp_ms is not None else self._current_timestamp_ms())
            t_now = self._time_from_ms(ts_ms)
            if t_now is None:
                return {tab: [] for tab in self.catalog_tabs}, 15.0
            out = {tab: [] for tab in self.catalog_tabs}
            for tab_name, entries in self.base_catalog.items():
                for raw in entries:
                    obj = self._resolve_target(raw.get("id")) or self._resolve_target(raw.get("name"))
                    if obj is None:
                        continue
                    try:
                        alt, _, _ = self.observer.at(t_now).observe(obj).apparent().altaz()
                    except Exception:
                        continue
                    if float(alt.degrees) >= 0:
                        row = copy.deepcopy(raw)
                        row["altitude_deg"] = round(float(alt.degrees), 2)
                        out[tab_name].append(row)
                out[tab_name].sort(key=lambda x: float(x.get("altitude_deg", -999.0)), reverse=True)
            return out, 30.0

    def tick(self, shared_data):
        with self.lock:
            now = time.monotonic()
            dt = 0.04 if self.last_tick_at <= 0 else _clamp(now - self.last_tick_at, 0.001, 0.25)
            self.last_tick_at = now
            if not self.ready:
                return

            joy = dict((shared_data or {}).get("joystick", {}))
            jx = float(joy.get("x", 0.0) or 0.0)
            jy = float(joy.get("y", 0.0) or 0.0)
            self.joystick_ra_offset_deg = _clamp(self.joystick_ra_offset_deg + (jx * 0.7 * dt), -12.0, 12.0)
            self.joystick_dec_offset_deg = _clamp(self.joystick_dec_offset_deg + ((-jy) * 0.7 * dt), -12.0, 12.0)

            self.focus_target_step = self.focus_best_step if self.focus_mode == "auto" else self.focus_manual_ticks
            if self.hardware_ready and self.focus_ready and (now - self.last_focus_send_s) >= 0.08:
                self.motor_link.send_command(1, "FOC", int(round(self.focus_target_step)))
                self.last_focus_send_s = now

            if not self.tracking_enabled:
                return

            mission = dict((shared_data or {}).get("mission", {}))
            target_id = mission.get("target", "STANDBY")
            if str(target_id).upper() == "STANDBY":
                return
            ts_ms = int((shared_data or {}).get("timestamp") or self._current_timestamp_ms())
            hadec = self._target_hadec(target_id, ts_ms)
            if hadec is None:
                return

            ha_deg, dec_deg = self.new_ha_dec(*hadec)
            motor_flags = dict((shared_data or {}).get("motor", {}))
            if bool(motor_flags.get("tracking_with_camera", True)):
                ha_deg += self.tracking_ra_offset_deg
                dec_deg += self.tracking_dec_offset_deg

            ha_deg += self.joystick_ra_offset_deg
            dec_deg += self.joystick_dec_offset_deg
            ha_deg = _clamp(ha_deg, -90.0, 90.0)
            dec_deg = _clamp(dec_deg, -180.0, 180.0)

            self.last_target_id = str(target_id)
            self.last_ha_deg = ha_deg
            self.last_dec_deg = dec_deg
            self.last_ra_step = int(round(ha_deg * self.ra_steps_per_degree))
            self.last_dec_step = int(round(dec_deg * self.dec_steps_per_degree))

            if (now - self.last_track_send_s) >= 0.04:
                self.motor_link.send_command(1, "RA", self.last_ra_step)
                self.motor_link.send_command(1, "DEC", self.last_dec_step)
                self.last_track_send_s = now

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
                    "ra_offset_deg": round(float(self.tracking_ra_offset_deg), 6),
                    "dec_offset_deg": round(float(self.tracking_dec_offset_deg), 6),
                    "joy_ra_offset_deg": round(float(self.joystick_ra_offset_deg), 6),
                    "joy_dec_offset_deg": round(float(self.joystick_dec_offset_deg), 6),
                },
                "gates": {
                    "hardware_ready": bool(self.hardware_ready),
                    "focus_ready": bool(self.focus_ready),
                    "tracking_enabled": bool(self.tracking_enabled),
                },
                "motor_link": self.motor_link.status(),
                "ui_exchange_size": len(self.ui_exchange),
                "last_tick_at": self.last_tick_at,
            }

    def close(self):
        self.motor_link.close()
