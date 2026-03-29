import argparse
import asyncio
import copy
import json
import os
import re
import ssl
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from aiohttp import WSMsgType, web
import zmq
try:
    import zwoasi as asi
except ImportError:
    asi = None
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MOVEMENT_SRC_DIR = PROJECT_ROOT / "Software" / "Movement" / "src"
if str(MOVEMENT_SRC_DIR) not in sys.path:
    sys.path.append(str(MOVEMENT_SRC_DIR))

try:
    from astronomic_operator import AstronomicOperator
except Exception as exc:
    AstronomicOperator = None
    print(f"Astronomic operator import failed: {exc}")


SSL_CERT = "cert.pem"
SSL_KEY = "key.pem"
ASI_LIB = "/usr/lib/libASICamera2.so"
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
JPEG_QUALITY = 60
BRIDGE_PIN = "1234"
CAPTURE_ROOT = Path("startrack_img")
VISION_INIT_SECONDS = 8.0
EXCHANGE_LOG_FILE = Path(__file__).resolve().parent / "exchange.log"
EXCHANGE_LOG_LIMIT = 2000
BACKGROUND_LOOP_INTERVAL_S = 0.05
CALIBRATION_COMMAND_TIMEOUT_S = 45.0

CATALOG = {
    "tab-solar": [
        {"id": "sun", "name": "SUN", "icon": "☀️"},
        {"id": "moon", "name": "MOON", "icon": "🌙"},
        {"id": "mercury", "name": "MERCURY", "icon": "🟡"},
        {"id": "venus", "name": "VENUS", "icon": "⚪"},
        {"id": "mars", "name": "MARS", "icon": "🏖️"},
        {"id": "jupiter barycenter", "name": "JUPITER", "icon": "🏀"},
        {"id": "saturn barycenter", "name": "SATURNE", "icon": "🪐"},
        {"id": "uranus barycenter", "name": "URANUS", "icon": "🔵"},
        {"id": "neptune barycenter", "name": "NEPTUNE", "icon": "⚓"}
    ],
    "tab-stars": [
        {"id": "32349", "name": "SIRIUS", "icon": "🐩"},
        {"id": "69673", "name": "ARCTURUS", "icon": "🪁"},
        {"id": "91262", "name": "VEGA", "icon": "🎻"},
        {"id": "24608", "name": "CAPELLA", "icon": "🐓"},
        {"id": "24436", "name": "RIGEL", "icon": "🔵"},
        {"id": "37279", "name": "PROCYON", "icon": "🐶"},
        {"id": "27989", "name": "BETELGEUSE", "icon": "🔴"},
        {"id": "97649", "name": "ALTAIR", "icon": "🦅"},
        {"id": "21421", "name": "ALDEBARAN", "icon": "🐂"},
        {"id": "65474", "name": "SPICA", "icon": "♍"},
        {"id": "80763", "name": "ANTARES", "icon": "🔴"},
        {"id": "11767", "name": "POLARIS", "icon": "❄️"},
        {"id": "37826", "name": "POLLUX", "icon": "👬"},
        {"id": "102098", "name": "DENEB", "icon": "🦢"},
        {"id": "49669", "name": "REGULUS", "icon": "🦁"},
        {"id": "36850", "name": "CASTOR", "icon": "💫"},
        {"id": "25336", "name": "BELLATRIX", "icon": "🔵"},
        {"id": "26311", "name": "ALNILAM", "icon": "⚪"},
        {"id": "26727", "name": "ALNITAK", "icon": "🔵"},
        {"id": "9884", "name": "HAMAL", "icon": "🐑"},
        {"id": "8796", "name": "SHERATAN", "icon": "🐑"},
        {"id": "15863", "name": "MIRFAK", "icon": "🤺"},
        {"id": "72607", "name": "KOCHAB", "icon": "🧸"},
        {"id": "87833", "name": "ELTANIN", "icon": "🐉"},
        {"id": "95947", "name": "ALBIREO", "icon": "💫"},
        {"id": "113368", "name": "FOMALHAUT", "icon": "🐟"},
        {"id": "30438", "name": "ADHARA", "icon": "🐩"},
        {"id": "25428", "name": "ELNATH", "icon": "🐂"},
        {"id": "62956", "name": "ALIOTH", "icon": "🐻"},
        {"id": "54061", "name": "DUBHE", "icon": "🟠"},
        {"id": "30324", "name": "WEZEN", "icon": "🐩"},
        {"id": "67301", "name": "ALKAID", "icon": "🔵"},
        {"id": "25110", "name": "MENKALINAN", "icon": "🐓"},
        {"id": "33579", "name": "ALHENA", "icon": "👬"},
        {"id": "29434", "name": "MIRZAM", "icon": "🔵"},
        {"id": "46390", "name": "ALPHARD", "icon": "🌌"},
        {"id": "3419", "name": "DIPHDA", "icon": "🐳"},
        {"id": "5447", "name": "MIRACH", "icon": "🔴"},
        {"id": "8362", "name": "ALPHERAT", "icon": "⭐"},
        {"id": "92855", "name": "NUNKI", "icon": "🏹"},
        {"id": "27366", "name": "SAIPH", "icon": "⚪"},
        {"id": "86032", "name": "RASALHAGUE", "icon": "🐍"}
    ],
    "tab-deep": [
        {"id": "M31", "name": "ANDROMEDA", "icon": "🌌", "ra_hours": 0.712, "dec_degrees": 41.269},
        {"id": "M32", "name": "M32 (COMPANION GALAXY)", "icon": "🔹", "ra_hours": 0.713, "dec_degrees": 40.865},
        {"id": "M110", "name": "M110 (COMPANION GALAXY)", "icon": "🌀", "ra_hours": 0.673, "dec_degrees": 41.685},
        {"id": "M34", "name": "M34 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 2.708, "dec_degrees": 42.75},
        {"id": "M36", "name": "M36 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 5.6, "dec_degrees": 34.14},
        {"id": "M37", "name": "M37 (OPEN CLUSTER)", "icon": "🌟", "ra_hours": 5.87, "dec_degrees": 32.55},
        {"id": "M38", "name": "M38 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 5.48, "dec_degrees": 35.85},
        {"id": "M41", "name": "M41 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 6.767, "dec_degrees": -20.733},
        {"id": "M42", "name": "ORION NEBULA", "icon": "💨", "ra_hours": 5.59, "dec_degrees": -5.45},
        {"id": "M44", "name": "M44 (BEEHIVE CLUSTER)", "icon": "✨", "ra_hours": 8.667, "dec_degrees": 19.667},
        {"id": "M45", "name": "PLEIADES", "icon": "✴️", "ra_hours": 3.783, "dec_degrees": 24.117},
        {"id": "M46", "name": "M46 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 7.7, "dec_degrees": -14.817},
        {"id": "M47", "name": "M47 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 7.617, "dec_degrees": -14.483},
        {"id": "M48", "name": "M48 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 8.217, "dec_degrees": -5.8},
        {"id": "M50", "name": "M50 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 7.033, "dec_degrees": -8.333},
        {"id": "M51", "name": "M51 (WHIRLPOOL GALAXY)", "icon": "🌌", "ra_hours": 13.5, "dec_degrees": 47.2},
        {"id": "M52", "name": "M52 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 23.267, "dec_degrees": 61.6},
        {"id": "M57", "name": "M57 (RING NEBULA)", "icon": "✨", "ra_hours": 18.9, "dec_degrees": 33.033},
        {"id": "M65", "name": "M65 (GALAXY)", "icon": "🌀", "ra_hours": 11.3, "dec_degrees": 13.083},
        {"id": "M66", "name": "M66 (GALAXY)", "icon": "🌀", "ra_hours": 11.333, "dec_degrees": 12.983},
        {"id": "M81", "name": "M81 (GALAXY)", "icon": "🌀", "ra_hours": 9.933, "dec_degrees": 69.067},
        {"id": "M82", "name": "M82 (GALAXY)", "icon": "🌀", "ra_hours": 9.933, "dec_degrees": 69.683},
        {"id": "M92", "name": "M92 (GLOBULAR CLUSTER)", "icon": "✨", "ra_hours": 17.283, "dec_degrees": 43.133},
        {"id": "M13", "name": "M13 (GLOBULAR CLUSTER)", "icon": "✨", "ra_hours": 16.7, "dec_degrees": 36.467},
        {"id": "M3", "name": "M3 (GLOBULAR CLUSTER)", "icon": "✨", "ra_hours": 13.7, "dec_degrees": 28.383},
        {"id": "M27", "name": "M27 (DUMBBELL NEBULA)", "icon": "✨", "ra_hours": 19.983, "dec_degrees": 22.717},
        {"id": "M56", "name": "M56 (GLOBULAR CLUSTER)", "icon": "✨", "ra_hours": 19.283, "dec_degrees": 30.183},
        {"id": "M71", "name": "M71 (GLOBULAR CLUSTER)", "icon": "✨", "ra_hours": 19.9, "dec_degrees": 18.783},
        {"id": "M103", "name": "M103 (OPEN CLUSTER)", "icon": "✨", "ra_hours": 1.533, "dec_degrees": 60.7}
    ],
    "tab-sat": [
        {"id": "ISS", "name": "ISS", "icon": "🛰️"},
        {"id": "HST", "name": "HUBBLE", "icon": "🔭"}
    ]
}


def utc_now_iso():
    return datetime.utcnow().replace(microsecond=0).isoformat()


def clamp(value, low, high):
    return max(low, min(high, value))


def _clamp_float(value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = low
    return max(float(low), min(float(high), value))


def parse_datetime_to_ms(value):
    if not value:
        return None
    try:
        normalized = value.replace("Z", "+00:00")
        return int(datetime.fromisoformat(normalized).timestamp() * 1000)
    except (TypeError, ValueError):
        return None


def sanitize_fs_name(value, fallback="UNKNOWN"):
    text = str(value or "").strip()
    if not text:
        text = fallback
    text = re.sub(r"[\\/:*?\"<>|]+", "_", text)
    text = re.sub(r"\s+", "_", text)
    return text[:64] or fallback


def detect_arduino_port(default_port):
    """
    Detect a likely Arduino serial port across Linux/Windows/macOS.
    Falls back to `default_port` if no suitable candidate is found.
    """
    if list_ports is None:
        return default_port

    try:
        ports = list(list_ports.comports())
    except Exception:
        return default_port

    if not ports:
        return default_port

    def _score(port_info):
        text = " ".join(
            [
                str(getattr(port_info, "device", "")),
                str(getattr(port_info, "description", "")),
                str(getattr(port_info, "manufacturer", "")),
                str(getattr(port_info, "hwid", "")),
            ]
        ).lower()
        score = 0
        if "arduino" in text:
            score += 20
        if "ch340" in text or "wch" in text:
            score += 15
        if "usb serial" in text or "ttyusb" in text or "ttyacm" in text:
            score += 10
        if str(getattr(port_info, "device", "")).upper().startswith("COM"):
            score += 5
        return score

    best = sorted(ports, key=_score, reverse=True)[0]
    return str(getattr(best, "device", default_port) or default_port)


class SystemState:
    def __init__(self):
        self.clients = {}
        self.client_seq = 0
        self.commander_ws = None
        self.commander_queue = deque()
        self.pending_requests = {}
        self.request_seq = 0
        self.mute_notifications = False
        self.emergency_stop = False

        self.joystick_owner = None
        self.joystick_expires_at = None
        self.joystick_queue = deque()
        self.action_expires_at = None
        self.last_action_before_temp = "--"

        now_ms = int(time.time() * 1000)
        self.clock_base_ms = now_ms
        self.clock_synced_at = time.monotonic()
        self.clock_was_explicitly_synced = False
        self.catalog = copy.deepcopy(CATALOG)
        self.exchange_log = deque(maxlen=EXCHANGE_LOG_LIMIT)

        self.data = {
            "gps": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "joystick": {"x": 0.0, "y": 0.0},
            "mission": {"target": "STANDBY", "action": "--"},
            "timestamp": now_ms,
            "datetime": utc_now_iso(),
            "camera": {"exposure": 30000, "gain": 150, "type": "auto"},
            "motor": {
                "tracking_with_camera": True,
                "compensate_earth_rotation": True,
            },
            "runtime_modes": {
                "camera_mode": "hardware",
                "simulate_arduino": False,
            },
            "focus": {"mode": "auto", "manual_value": 50, "manual_ticks": 5058},
            "capture": {
                "video_duration": 10,
                "last_photo_at": "",
                "last_photo_path": "",
                "last_video_request_at": "",
                "last_video_duration": 10,
                "last_video_path": "",
                "recording": False,
                "last_error": "",
            },
            "vision_status": {
                "ready": False,
                "stage": "initializing",
                "focus_score": 0.0,
                "mode": "auto",
                "manual_value": 50,
                "best_step": 0.0,
                "current_step": 0.0,
                "dx": 0.0,
                "dy": 0.0,
                "tracking_available": False,
                "initialized_at": "",
            },
            "astronomic_status": {
                "ready": False,
                "time_synced": False,
                "has_location": False,
                "skyfield_ready": False,
                "message": "Waiting for GPS and time sync.",
            },
            "arduino_status": {
                "available": False,
                "system_state": "UNKNOWN",
                "calibration_done": False,
                "last_response": "",
                "last_debug_line": "",
                "motor_state": {"RA": "UNKNOWN", "DEC": "UNKNOWN", "FOC": "UNKNOWN"},
            },
            "system": {
                "hardware_calibrated": False,
                "vision_ready": False,
                "astronomic_ready": False,
                "controls_ready": False,
                "catalog_ready": False,
                "message": "Waiting for initialization.",
            },
        }

    def next_request_id(self):
        self.request_seq += 1
        return self.request_seq

    def register(self, ws, ip):
        self.client_seq += 1
        self.clients[ws] = {
            "id": self.client_seq,
            "name": "Anon",
            "role": "OBSERVER",
            "ip": ip or "unknown",
        }

    def unregister(self, ws):
        if ws not in self.clients:
            return

        self.remove_from_commander_queue(ws)
        self.remove_from_joystick_queue(ws)
        self.clear_pending_requests_for(ws)

        was_commander = ws == self.commander_ws
        was_joystick_owner = ws == self.joystick_owner

        del self.clients[ws]

        if was_commander:
            self.commander_ws = None
            self.promote_next_commander()

        if was_joystick_owner:
            self.joystick_owner = None
            self.joystick_expires_at = None
            self.data["joystick"] = {"x": 0.0, "y": 0.0}

    def clear_pending_requests_for(self, ws):
        stale = []
        for req_id, request in self.pending_requests.items():
            if request["ws"] == ws:
                stale.append(req_id)
        for req_id in stale:
            del self.pending_requests[req_id]

    def has_pending_request(self, ws, kinds=None):
        for request in self.pending_requests.values():
            if request["ws"] != ws:
                continue
            if kinds is None or request["kind"] in kinds:
                return True
        return False

    def remove_from_commander_queue(self, ws):
        self.commander_queue = deque(
            queued_ws for queued_ws in self.commander_queue if queued_ws != ws
        )

    def remove_from_joystick_queue(self, ws):
        self.joystick_queue = deque(
            entry for entry in self.joystick_queue if entry["ws"] != ws
        )

    def client_name(self, ws):
        client = self.clients.get(ws)
        return client["name"] if client else "Unknown"

    def client_id(self, ws):
        client = self.clients.get(ws)
        return client["id"] if client else None

    def client_ws_by_id(self, client_id):
        for ws, client in self.clients.items():
            if client.get("id") == client_id:
                return ws
        return None

    def client_role(self, ws):
        client = self.clients.get(ws)
        return client["role"] if client else "OBSERVER"

    def client_meta(self, ws):
        return {
            "client_id": self.client_id(ws),
            "name": self.client_name(ws),
            "role": self.client_role(ws),
        }

    def is_commander(self, ws):
        return ws is not None and ws == self.commander_ws

    def is_bridge(self, ws):
        return self.client_role(ws) == "BRIDGE"

    def can_admin(self, ws):
        return self.is_commander(ws) or self.is_bridge(ws)

    def set_client_name(self, ws, name):
        if ws in self.clients and name:
            self.clients[ws]["name"] = name[:32]

    def set_commander(self, ws, name=None):
        if ws not in self.clients or self.is_bridge(ws):
            return False

        if self.commander_ws and self.commander_ws in self.clients:
            self.clients[self.commander_ws]["role"] = "OBSERVER"

        self.commander_ws = ws
        self.clients[ws]["role"] = "COMMANDER"
        if name:
            self.clients[ws]["name"] = name[:32]
        self.remove_from_commander_queue(ws)
        return True

    def promote_next_commander(self):
        while self.commander_queue:
            next_ws = self.commander_queue.popleft()
            if next_ws in self.clients and not self.is_bridge(next_ws):
                self.set_commander(next_ws, self.client_name(next_ws))
                return next_ws

        for ws, client in self.clients.items():
            if client["role"] != "BRIDGE":
                self.set_commander(ws, client["name"])
                return ws

        return None

    def has_pending_joystick_request(self):
        return any(
            request["kind"] in {"JOYSTICK_REQUEST", "JOYSTICK_QUEUE_REQUEST"}
            for request in self.pending_requests.values()
        )

    def queue_joystick(self, ws, duration):
        if ws not in self.clients:
            return False
        if any(entry["ws"] == ws for entry in self.joystick_queue):
            return False
        self.joystick_queue.append({"ws": ws, "duration": duration})
        return True

    def pop_next_joystick(self):
        while self.joystick_queue:
            entry = self.joystick_queue.popleft()
            if entry["ws"] in self.clients:
                return entry
        return None

    def serialize_joystick_queue(self):
        serialized = []
        for entry in self.joystick_queue:
            ws = entry["ws"]
            if ws not in self.clients:
                continue
            serialized.append(
                {
                    "client_id": self.client_id(ws),
                    "name": self.client_name(ws),
                    "duration": entry["duration"],
                }
            )
        return serialized

    def move_joystick_queue_entry(self, client_id, direction):
        entries = [entry for entry in self.joystick_queue if entry["ws"] in self.clients]
        index = next(
            (
                idx
                for idx, entry in enumerate(entries)
                if self.client_id(entry["ws"]) == client_id
            ),
            None,
        )
        if index is None:
            return False
        new_index = clamp(index + direction, 0, len(entries) - 1)
        if new_index == index:
            return False
        entries[index], entries[new_index] = entries[new_index], entries[index]
        self.joystick_queue = deque(entries)
        return True

    def remove_joystick_queue_entry_by_id(self, client_id):
        removed_ws = None
        filtered = []
        for entry in self.joystick_queue:
            ws = entry["ws"]
            if ws not in self.clients:
                continue
            if self.client_id(ws) == client_id and removed_ws is None:
                removed_ws = ws
                continue
            filtered.append(entry)
        self.joystick_queue = deque(filtered)
        return removed_ws

    def adjust_joystick_queue_duration(self, client_id, delta_seconds):
        updated = False
        entries = []
        for entry in self.joystick_queue:
            ws = entry["ws"]
            if ws not in self.clients:
                continue
            if self.client_id(ws) == client_id:
                entry = {
                    **entry,
                    "duration": clamp(int(entry["duration"]) + int(delta_seconds), 10, 3600),
                }
                updated = True
            entries.append(entry)
        self.joystick_queue = deque(entries)
        return updated

    def sync_clock(self, timestamp_ms=None, datetime_value=None, explicit=False):
        derived_ms = parse_datetime_to_ms(datetime_value)
        effective_ms = timestamp_ms or derived_ms or int(time.time() * 1000)
        self.clock_base_ms = int(effective_ms)
        self.clock_synced_at = time.monotonic()
        if explicit:
            self.clock_was_explicitly_synced = True
        self.update_clock()

    def update_clock(self):
        elapsed_ms = int((time.monotonic() - self.clock_synced_at) * 1000)
        current_ms = self.clock_base_ms + elapsed_ms
        self.data["timestamp"] = current_ms
        self.data["datetime"] = datetime.fromtimestamp(
            current_ms / 1000
        ).isoformat(timespec="seconds")

    def joystick_remaining(self):
        if self.joystick_owner is None or self.joystick_expires_at is None:
            return None
        remaining = int(round(self.joystick_expires_at - time.time()))
        return max(0, remaining)

    def adjust_joystick_owner_duration(self, delta_seconds):
        if self.joystick_owner is None or self.joystick_expires_at is None:
            return False
        remaining = self.joystick_remaining()
        if remaining is None:
            return False
        next_remaining = clamp(int(remaining) + int(delta_seconds), 10, 3600)
        self.joystick_expires_at = time.time() + next_remaining
        return True

    def set_temporary_action(self, action, duration_seconds):
        current_action = self.data["mission"]["action"]
        if current_action not in {"PHOTO", "VIDEO"}:
            self.last_action_before_temp = current_action
        self.data["mission"]["action"] = action
        self.action_expires_at = time.time() + max(0, duration_seconds)

    def clear_temporary_action_if_needed(self):
        if self.action_expires_at is None:
            return False
        if time.time() < self.action_expires_at:
            return False
        self.action_expires_at = None
        if self.data["mission"]["action"] in {"PHOTO", "VIDEO"}:
            self.data["mission"]["action"] = self.last_action_before_temp or "--"
        return True
        return False

    def serialize_pending_requests(self):
        serialized = []
        for request_id, request in self.pending_requests.items():
            if request["ws"] not in self.clients:
                continue
            payload = {
                "req_id": request_id,
                "kind": request["kind"],
                "name": self.client_name(request["ws"]),
            }
            if "sub_kind" in request:
                payload["sub_kind"] = request["sub_kind"]
            if "duration" in request:
                payload["duration"] = request["duration"]
            if "desc" in request:
                payload["desc"] = request["desc"]
            if "payload" in request:
                payload["payload"] = request["payload"]
            serialized.append(payload)
        return serialized

    def serialize_system_info(self, ws):
        users = [
            {"id": client["id"], "name": client["name"], "role": client["role"]}
            for client in self.clients.values()
        ]
        return {
            "type": "SYS_INFO",
            "my_role": self.client_role(ws),
            "my_name": self.client_name(ws),
            "i_have_joystick": ws == self.joystick_owner,
            "commander": self.client_name(self.commander_ws)
            if self.commander_ws
            else "NONE",
            "commander_id": self.client_id(self.commander_ws),
            "users": users,
            "commander_queue": [
                self.client_name(queued_ws)
                for queued_ws in self.commander_queue
                if queued_ws in self.clients
            ],
            "joystick_user": self.client_name(self.joystick_owner)
            if self.joystick_owner
            else "NONE",
            "joystick_user_id": self.client_id(self.joystick_owner),
            "joystick_owner_id": id(self.joystick_owner)
            if self.joystick_owner
            else None,
            "joystick_queue": self.serialize_joystick_queue(),
            "joystick_remaining": self.joystick_remaining(),
            "mute_notifications": self.mute_notifications,
            "pending_requests": self.serialize_pending_requests()
            if self.is_commander(ws)
            else [],
        }

    def record_exchange(self, direction, payload, ws=None, delivered=True):
        packet = payload if isinstance(payload, dict) else {"raw": str(payload)}
        event = {
            "ts": utc_now_iso(),
            "direction": str(direction or "unknown"),
            "type": packet.get("type", "UNKNOWN"),
            "delivered": bool(delivered),
            "client": self.client_meta(ws) if ws is not None else None,
            "packet": copy.deepcopy(packet),
        }
        self.exchange_log.append(event)
        try:
            with EXCHANGE_LOG_FILE.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def exchange_snapshot(self, limit=200):
        amount = clamp(int(limit), 1, EXCHANGE_LOG_LIMIT)
        return list(self.exchange_log)[-amount:]


state = SystemState()


class VisionCoordinator:
    TOTAL_FOCUS_STEPS = 10116.0

    def __init__(self, init_seconds=VISION_INIT_SECONDS):
        self.init_seconds = max(3.0, float(init_seconds))
        self.active = False
        self.ready = False
        self.stage = "waiting_hardware_calibration"
        self.initialized_at = ""
        self.focus_score = 0.0
        self.mode = state.data["focus"]["mode"]
        self.manual_value = int(state.data["focus"]["manual_value"])
        self.best_focus_score = -1.0
        self.best_focus_step = 0.0
        self.current_focus_step = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.frame_width = VIDEO_WIDTH
        self.frame_height = VIDEO_HEIGHT
        self.tracking_available = False

        # Autofocus sweep state
        self.pass_speeds = [500.0, 250.0, 120.0]
        self.pass_ranges = [
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
        ]
        self.current_pass = 0
        self.pending_focus_command = None
        self.sweep_samples = []
        self.sweep_started_at = 0.0
        self.sweep_start_step = 0.0
        self.sweep_target_step = 0.0
        self.last_sweep_duration_s = 0.0
        self.last_sweep_measured_speed = 0.0
        self.last_sweep_sample_count = 0
        self.last_focus_error = ""
        self.autofocus_retry_count = 0

    def _manual_to_step(self):
        return (float(self.manual_value) / 100.0) * self.TOTAL_FOCUS_STEPS

    def _update_tracking_error(self, frame):
        if frame is None:
            self.dx = 0.0
            self.dy = 0.0
            self.tracking_available = False
            return

        self.frame_height, self.frame_width = frame.shape[:2]
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
            _, mask = cv2.threshold(
                blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                self.dx = 0.0
                self.dy = 0.0
                self.tracking_available = False
                return

            largest = max(contours, key=cv2.contourArea)
            (cx, cy), _ = cv2.minEnclosingCircle(largest)
            center_x = int(cx)
            center_y = int(cy)
            self.dx = float((self.frame_width // 2) - center_x)
            self.dy = float((self.frame_height // 2) - center_y)
            self.tracking_available = True
        except Exception:
            self.dx = 0.0
            self.dy = 0.0
            self.tracking_available = False

    def update_focus_settings(self, mode, manual_value):
        if mode in {"auto", "manual"}:
            self.mode = mode
        try:
            self.manual_value = clamp(int(manual_value), 0, 100)
        except (TypeError, ValueError):
            pass

        if self.mode == "manual":
            self.current_focus_step = self._manual_to_step()
        elif self.ready:
            self.current_focus_step = self.best_focus_step

    def update_from_frame(self, frame):
        self._update_tracking_error(frame)
        if frame is not None:
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.focus_score = float(np.var(cv2.Laplacian(gray, cv2.CV_64F)))
            except Exception:
                self.focus_score = 0.0

        # Collect focus score samples only while the sweep command is active.
        if self.active and self.pending_focus_command == "SWEEP":
            self.sweep_samples.append((time.monotonic(), float(self.focus_score)))

        if not self.active:
            self.stage = "waiting_hardware_calibration"
            if self.mode == "manual":
                self.current_focus_step = self._manual_to_step()

    def start_focus_calibration(self):
        self.active = True
        self.ready = False
        self.stage = "focus_prepare_pass_1"
        self.initialized_at = ""
        self.best_focus_score = -1.0
        self.best_focus_step = 0.0
        self.current_focus_step = 0.0
        self.current_pass = 0
        self.pending_focus_command = None
        self.sweep_samples = []
        self.sweep_started_at = 0.0
        self.sweep_start_step = 0.0
        self.sweep_target_step = 0.0
        self.last_sweep_duration_s = 0.0
        self.last_sweep_measured_speed = 0.0
        self.last_sweep_sample_count = 0
        self.last_focus_error = ""
        self.autofocus_retry_count = 0
        self.pass_ranges = [
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
        ]

    def _set_next_pass_range(self, pass_index, best_step):
        if pass_index == 0:
            pad = 280.0
        elif pass_index == 1:
            pad = 90.0
        else:
            return
        low = max(0.0, float(best_step) - pad)
        high = min(self.TOTAL_FOCUS_STEPS, float(best_step) + pad)
        self.pass_ranges[pass_index + 1] = (low, high)

    def _send_focus_position(self, link, target_step, stage_name):
        sent = link.send_command(1, "FOC", int(round(target_step)))
        if sent:
            self.pending_focus_command = "REWIND" if "rewind" in stage_name else "LOCK"
            self.stage = stage_name
            self.current_focus_step = float(target_step)
        return sent

    def _send_focus_sweep(self, link, start_step, target_step, speed):
        sent = link.send_command(2, "FOC", int(round(target_step)), float(speed))
        if sent:
            self.pending_focus_command = "SWEEP"
            self.stage = f"focus_sweep_pass_{self.current_pass + 1}"
            self.sweep_samples = []
            self.sweep_started_at = time.monotonic()
            self.sweep_start_step = float(start_step)
            self.sweep_target_step = float(target_step)
            self.current_focus_step = float(start_step)
        return sent

    def _finalize_sweep(self):
        ended_at = time.monotonic()
        duration = max(0.001, ended_at - self.sweep_started_at)
        self.last_sweep_duration_s = duration
        distance = abs(self.sweep_target_step - self.sweep_start_step)
        self.last_sweep_measured_speed = distance / duration

        if not self.sweep_samples:
            self.last_focus_error = "No focus samples captured during sweep."
            return None

        direction = 1.0 if self.sweep_target_step >= self.sweep_start_step else -1.0
        best_step = self.sweep_start_step
        best_score = -1.0

        for sample_ts, score in self.sweep_samples:
            ratio = _clamp_float((sample_ts - self.sweep_started_at) / duration, 0.0, 1.0)
            estimated_step = self.sweep_start_step + direction * (self.last_sweep_measured_speed * duration * ratio)
            if direction > 0:
                estimated_step = min(self.sweep_target_step, max(self.sweep_start_step, estimated_step))
            else:
                estimated_step = max(self.sweep_target_step, min(self.sweep_start_step, estimated_step))

            if score >= best_score:
                best_score = score
                best_step = estimated_step

        self.last_sweep_sample_count = len(self.sweep_samples)
        self.best_focus_score = max(self.best_focus_score, best_score)
        self.best_focus_step = float(best_step)
        self.current_focus_step = float(best_step)
        return best_step

    def handle_arduino_events(self, events, link):
        if not self.active:
            return

        for event in events:
            if event.get("kind") != "ack":
                continue
            if str(event.get("motor", "")).upper() != "FOC":
                continue

            code = event.get("code")
            action = event.get("action")

            # Focus command rejected by firmware -> retry a limited number of times.
            if code == 0:
                failed_command = self.pending_focus_command
                self.autofocus_retry_count += 1
                self.pending_focus_command = None
                if self.autofocus_retry_count <= 3:
                    if failed_command == "LOCK":
                        self.stage = f"focus_lock_retry_{self.autofocus_retry_count}"
                        self._send_focus_position(link, self.best_focus_step, "focus_lock_best")
                    else:
                        self.stage = f"focus_prepare_pass_{self.current_pass + 1}"
                else:
                    self.last_focus_error = "Focus command rejected repeatedly."
                    self.stage = "focus_error"
                    self.active = False
                continue

            if code != 2:
                continue

            # Done for rewind/start positioning.
            if action == 1 and self.pending_focus_command == "REWIND":
                self.autofocus_retry_count = 0
                self.pending_focus_command = None
                start_step, target_step = self.pass_ranges[self.current_pass]
                speed = self.pass_speeds[self.current_pass]
                self._send_focus_sweep(link, start_step, target_step, speed)
                continue

            # Done for sweep move.
            if action == 2 and self.pending_focus_command == "SWEEP":
                self.autofocus_retry_count = 0
                self.pending_focus_command = None
                best_step = self._finalize_sweep()
                if best_step is None:
                    self.stage = "focus_error"
                    self.active = False
                    continue

                if self.current_pass < 2:
                    self._set_next_pass_range(self.current_pass, best_step)
                    self.current_pass += 1
                    self.stage = f"focus_prepare_pass_{self.current_pass + 1}"
                else:
                    self._send_focus_position(link, best_step, "focus_lock_best")
                continue

            # Done for final lock.
            if action == 1 and self.pending_focus_command == "LOCK":
                self.autofocus_retry_count = 0
                self.pending_focus_command = None
                self.ready = True
                self.stage = "tracking"
                self.active = True
                self.initialized_at = utc_now_iso()
                if self.mode == "auto":
                    self.current_focus_step = self.best_focus_step
                else:
                    self.current_focus_step = self._manual_to_step()

    def tick_autofocus(self, link):
        if not self.active or self.ready or self.pending_focus_command is not None:
            return

        if self.current_pass >= len(self.pass_ranges):
            return

        if self.stage.startswith("focus_prepare_pass_"):
            start_step, _ = self.pass_ranges[self.current_pass]
            self._send_focus_position(
                link,
                start_step,
                f"focus_rewind_pass_{self.current_pass + 1}",
            )

    def to_payload(self):
        return {
            "active": bool(self.active),
            "ready": self.ready,
            "stage": self.stage,
            "focus_score": round(float(self.focus_score), 2),
            "mode": self.mode,
            "manual_value": self.manual_value,
            "best_step": round(float(self.best_focus_step), 2),
            "current_step": round(float(self.current_focus_step), 2),
            "dx": round(float(self.dx), 2),
            "dy": round(float(self.dy), 2),
            "tracking_available": bool(self.tracking_available),
            "frame_width": int(self.frame_width),
            "frame_height": int(self.frame_height),
            "initialized_at": self.initialized_at,
            "pass_index": int(self.current_pass + 1),
            "sweep_duration_s": round(float(self.last_sweep_duration_s), 4),
            "sweep_speed_ticks_per_s": round(float(self.last_sweep_measured_speed), 3),
            "sweep_samples": int(self.last_sweep_sample_count),
            "last_error": self.last_focus_error,
            "focus_range_ticks": int(self.TOTAL_FOCUS_STEPS),
        }


def build_capture_paths(target_id, capture_kind):
    now = datetime.now()
    day_folder = now.strftime("%Y-%m-%d")
    mission_folder = sanitize_fs_name(target_id, fallback="STANDBY")
    stamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    prefix = "Photo" if capture_kind == "photo" else "Video"
    extension = ".jpg" if capture_kind == "photo" else ".mp4"

    base_dir = CAPTURE_ROOT / day_folder / mission_folder
    filename = f"{prefix}_{mission_folder}_{stamp}{extension}"
    return base_dir, base_dir / filename


class RuntimeOrchestrator:
    def __init__(self):
        self.hw_calibration_requested = False
        self.hw_calibration_requested_at = 0.0
        self.hw_calibration_accepted = False
        self.hw_calibrated = False
        self.hw_calibration_retries = 0
        self.hw_next_retry_at = 0.0
        self.focus_calibration_started = False
        self.focus_calibrated = False
        self.last_handshake_code = None

    def reset(self):
        self.__init__()


astro_runtime = None
vision_runtime = VisionCoordinator()
runtime_orchestrator = RuntimeOrchestrator()
catalog_refresh_deadline = 0.0
catalog_last_hash = ""


class CameraManager:
    def __init__(self, mode="hardware"):
        self.mode = mode
        self.frame = None
        self.raw_frame = None
        self.running = True
        self.camera = None
        self.video = None
        self.lock = threading.Lock()
        self.settings_lock = threading.Lock()
        self.recording_lock = threading.Lock()
        self.recording_thread = None
        self.recording_active = False
        self.last_recording_error = ""
        self.last_recording_path = ""
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        context = zmq.Context()
        self.zmq_socket = context.socket(zmq.PUB)
        self.zmq_socket.bind("ipc:///tmp/video_feed")

    def start(self):
        self.thread.start()

    def get_frame(self):
        with self.lock:
            return self.frame

    def get_raw_frame_copy(self):
        with self.lock:
            if self.raw_frame is None:
                return None
            return self.raw_frame.copy()

    def apply_camera_settings(self, settings):
        if self.camera is None or asi is None:
            return
        try:
            with self.settings_lock:
                self.camera.set_control_value(
                    asi.ASI_EXPOSURE, int(settings["exposure"]), auto=False
                )
                self.camera.set_control_value(
                    asi.ASI_GAIN, int(settings["gain"]), auto=False
                )
        except Exception as exc:
            print(f"Camera settings update failed: {exc}")

    def apply_focus_settings(self, focus):
        if self.camera is None or asi is None:
            return
        focus_mode = focus.get("mode", "auto")
        focus_value = clamp(int(focus.get("manual_value", 50)), 0, 100)
        try:
            with self.settings_lock:
                if hasattr(asi, "ASI_FOCUS"):
                    self.camera.set_control_value(
                        asi.ASI_FOCUS,
                        focus_value,
                        auto=(focus_mode == "auto"),
                    )
        except Exception as exc:
            print(f"Focus settings update failed: {exc}")

    def _init_hardware_camera(self):
        if asi is None:
            return False
        try:
            asi.init(ASI_LIB)
            if asi.get_num_cameras() <= 0:
                return False
            self.camera = asi.Camera(0)
            self.camera.set_roi(
                width=VIDEO_WIDTH,
                height=VIDEO_HEIGHT,
                bins=1,
                image_type=asi.ASI_IMG_RGB24,
            )
            self.apply_camera_settings(state.data["camera"])
            self.camera.start_video_capture()
            return True
        except Exception as exc:
            print(f"Hardware camera init failed: {exc}")
            self.camera = None
            return False

    def _init_simulation(self):
        if os.path.exists("simulation.mp4"):
            self.video = cv2.VideoCapture("simulation.mp4")

    def _capture_loop(self):
        print(f"[{self.mode.upper()}] Camera loop starting")

        if self.mode == "hardware" and not self._init_hardware_camera():
            self.mode = "sim"

        if self.mode == "sim":
            self._init_simulation()

        while self.running:
            frame = None

            if self.mode == "hardware" and self.camera is not None:
                try:
                    frame = self.camera.capture_video_frame()
                except Exception:
                    frame = None
            else:
                if self.video is not None:
                    ok, raw = self.video.read()
                    if not ok:
                        self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ok, raw = self.video.read()
                    if ok:
                        frame = cv2.resize(raw, (VIDEO_WIDTH, VIDEO_HEIGHT))
                if frame is None:
                    frame = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "NO VIDEO SOURCE",
                        (80, VIDEO_HEIGHT // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.5,
                        (0, 0, 255),
                        3,
                    )
                    cv2.putText(
                        frame,
                        datetime.now().strftime("%H:%M:%S"),
                        (80, VIDEO_HEIGHT // 2 + 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (255, 255, 255),
                        2,
                    )

            success, jpeg = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if success:
                with self.lock:
                    self.frame = jpeg.tobytes()
                    self.raw_frame = frame.copy()

            try:
                h, w = frame.shape[:2]
                payload = [
                    b"video_feed",
                    str(time.time()).encode("utf-8"),
                    f"{h},{w},3".encode("utf-8"),
                    frame.tobytes(),
                ]
                self.zmq_socket.send_multipart(payload, flags=zmq.NOBLOCK)
            except Exception:
                pass

            time.sleep(0.04)

    def capture_photo(self, output_path):
        frame = self.get_raw_frame_copy()
        if frame is None:
            return False, "No video frame available yet."

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(output_path), frame)
        if not ok:
            return False, "Failed to save photo."
        return True, ""

    def _record_video_worker(self, output_path, duration_seconds, fps):
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(
                str(output_path),
                fourcc,
                float(fps),
                (VIDEO_WIDTH, VIDEO_HEIGHT),
            )
            if not writer.isOpened():
                raise RuntimeError("Video writer could not be opened.")

            started = time.monotonic()
            interval = 1.0 / max(1.0, float(fps))
            while self.running and (time.monotonic() - started) < float(duration_seconds):
                frame = self.get_raw_frame_copy()
                if frame is not None:
                    if frame.shape[1] != VIDEO_WIDTH or frame.shape[0] != VIDEO_HEIGHT:
                        frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
                    writer.write(frame)
                time.sleep(interval)

            writer.release()
            with self.recording_lock:
                self.recording_active = False
                self.last_recording_path = str(output_path)
                self.last_recording_error = ""
        except Exception as exc:
            with self.recording_lock:
                self.recording_active = False
                self.last_recording_error = str(exc)

    def start_video_recording(self, output_path, duration_seconds, fps=20):
        with self.recording_lock:
            if self.recording_active:
                return False, "A video recording is already in progress."

            self.recording_active = True
            self.last_recording_error = ""
            self.last_recording_path = ""
            self.recording_thread = threading.Thread(
                target=self._record_video_worker,
                args=(Path(output_path), float(duration_seconds), float(fps)),
                daemon=True,
            )
            self.recording_thread.start()
            return True, ""

    def recording_status(self):
        with self.recording_lock:
            return {
                "active": bool(self.recording_active),
                "last_error": self.last_recording_error,
                "last_path": self.last_recording_path,
            }


cam = None


def record_exchange(direction, payload, ws=None, delivered=True):
    state.record_exchange(direction, payload, ws=ws, delivered=delivered)
    if astro_runtime is not None:
        try:
            astro_runtime.on_ui_packet(
                direction=direction,
                packet=payload,
                meta=state.client_meta(ws) if ws is not None else {},
            )
        except Exception:
            pass


async def safe_send(ws, payload):
    if ws is None or ws.closed:
        record_exchange("tx", payload, ws=ws, delivered=False)
        return False
    try:
        await ws.send_json(payload)
        record_exchange("tx", payload, ws=ws, delivered=True)
        return True
    except Exception as exc:
        print(f"WebSocket send failed: {exc}")
        record_exchange("tx", payload, ws=ws, delivered=False)
        return False


async def send_full_state(ws):
    await safe_send(
        ws,
        {
            "type": "FULL_STATE",
            "data": state.data,
            "catalog": state.catalog,
            "role": state.client_role(ws),
            "name": state.client_name(ws),
            "i_have_joystick": ws == state.joystick_owner,
            "pending_requests": state.serialize_pending_requests()
            if state.is_commander(ws)
            else [],
        },
    )


async def broadcast_system_info():
    for ws in list(state.clients.keys()):
        await safe_send(ws, state.serialize_system_info(ws))


async def broadcast_catalog():
    payload = {"type": "CATALOG_UPDATE", "catalog": state.catalog}
    for ws in list(state.clients.keys()):
        await safe_send(ws, payload)


async def broadcast_telemetry():
    payload = {"type": "TELEMETRY", "data": state.data}
    for ws in list(state.clients.keys()):
        await safe_send(ws, payload)


async def trigger_emergency_stop():
    if state.emergency_stop:
        return
    state.emergency_stop = True
    await broadcast_system_info()
    await broadcast_telemetry()
    for ws in list(state.clients.keys()):
        await safe_send(ws, {"type": "EMERGENCY_STOP"})
        try:
            await ws.close()
        except Exception:
            pass


async def notify_request(request_id):
    request = state.pending_requests.get(request_id)
    if request is None:
        return

    target = state.commander_ws
    if target is None:
        requester = request["ws"]
        await safe_send(
            requester,
            {"type": "TOAST", "msg": "No commander available to review the request."},
        )
        del state.pending_requests[request_id]
        return

    message = {
        "type": "NOTIFICATION",
        "req_id": request_id,
        "kind": request["kind"],
        "name": state.client_name(request["ws"]),
    }
    if "sub_kind" in request:
        message["sub_kind"] = request["sub_kind"]
    if "duration" in request:
        message["duration"] = request["duration"]
    if "desc" in request:
        message["desc"] = request["desc"]
    if "payload" in request:
        message["payload"] = request["payload"]

    if not await safe_send(target, message):
        requester = request["ws"]
        await safe_send(
            requester,
            {"type": "TOAST", "msg": "Commander is unavailable."},
        )
        del state.pending_requests[request_id]


async def grant_joystick(ws, duration=None, reason="approved"):
    previous_owner = state.joystick_owner

    if previous_owner and previous_owner != ws:
        state.joystick_owner = None
        state.joystick_expires_at = None
        state.data["joystick"] = {"x": 0.0, "y": 0.0}
        await safe_send(previous_owner, {"type": "JOYSTICK_REVOKED", "reason": reason})

    state.remove_from_joystick_queue(ws)
    state.joystick_owner = ws
    if state.is_commander(ws):
        state.joystick_expires_at = None
    else:
        granted_duration = clamp(int(duration or 30), 10, 3600)
        state.joystick_expires_at = time.time() + granted_duration
        duration = granted_duration

    await safe_send(
        ws,
        {
            "type": "JOYSTICK_GRANTED",
            "duration": duration,
            "remaining": state.joystick_remaining(),
        },
    )
    await broadcast_system_info()
    await broadcast_telemetry()


async def revoke_joystick(reason="revoked", advance_queue=True):
    owner = state.joystick_owner
    if owner is None:
        if advance_queue:
            await grant_next_joystick()
        return

    state.joystick_owner = None
    state.joystick_expires_at = None
    state.data["joystick"] = {"x": 0.0, "y": 0.0}

    await safe_send(owner, {"type": "JOYSTICK_REVOKED", "reason": reason})
    await broadcast_system_info()
    await broadcast_telemetry()

    if advance_queue:
        await grant_next_joystick()


async def grant_next_joystick():
    next_entry = state.pop_next_joystick()
    if next_entry is None:
        await broadcast_system_info()
        return
    await grant_joystick(
        next_entry["ws"], next_entry["duration"], reason="queue_advanced"
    )
    await safe_send(
        next_entry["ws"],
        {"type": "TOAST", "msg": "Joystick access moved to your turn."},
    )


async def manage_joystick_queue(ws, pkt):
    if not state.can_admin(ws):
        return

    action = str(pkt.get("action", "")).upper()
    client_id = pkt.get("client_id")
    try:
        client_id = int(client_id)
    except (TypeError, ValueError):
        client_id = None

    changed = False

    if action == "MOVE_UP" and client_id is not None:
        changed = state.move_joystick_queue_entry(client_id, -1)
    elif action == "MOVE_DOWN" and client_id is not None:
        changed = state.move_joystick_queue_entry(client_id, 1)
    elif action == "REMOVE" and client_id is not None:
        removed_ws = state.remove_joystick_queue_entry_by_id(client_id)
        changed = removed_ws is not None
        if removed_ws is not None:
            await safe_send(
                removed_ws,
                {"type": "TOAST", "msg": "You were removed from the joystick queue."},
            )
    elif action == "ADD_30" and client_id is not None:
        changed = state.adjust_joystick_queue_duration(client_id, 30)
    elif action == "SUB_30" and client_id is not None:
        changed = state.adjust_joystick_queue_duration(client_id, -30)

    if changed:
        await broadcast_system_info()


async def adjust_joystick_owner_duration(ws, pkt):
    if not state.can_admin(ws):
        return
    try:
        delta = clamp(int(pkt.get("delta", 0)), -300, 300)
    except (TypeError, ValueError):
        return
    if delta == 0:
        return
    if state.adjust_joystick_owner_duration(delta):
        await broadcast_system_info()


def is_command_allowed_during_init(packet_type):
    return packet_type in {
        "GPS_UPDATE",
        "SYNC_CLOCK",
        "TIMESTAMP",
        "DATETIME",
        "CAMERA_SETTINGS",
        "RUNTIME_MODES",
        "EMERGENCY_STOP",
    }


def refresh_system_status():
    astro_status = (
        astro_runtime.status()
        if astro_runtime is not None
        else {
            "ready": True,
            "time_synced": True,
            "has_location": True,
            "skyfield_ready": False,
            "message": "Astronomic runtime unavailable, using static fallback.",
        }
    )
    vision_status = vision_runtime.to_payload()
    arduino_status = (
        (astro_status.get("motor_link", {}) if isinstance(astro_status, dict) else {})
        or {}
    )

    hardware_calibrated = bool(runtime_orchestrator.hw_calibrated)
    focus_ready = bool(vision_status.get("ready", False))
    astro_ready = bool(astro_status.get("ready", False))

    controls_ready = bool(hardware_calibrated and focus_ready and astro_ready)
    if controls_ready:
        message = "All systems initialized."
    else:
        waiting = []
        if not hardware_calibrated:
            waiting.append("arduino calibration")
        if hardware_calibrated and not vision_status["ready"]:
            waiting.append("vision autofocus")
        if not astro_status.get("has_location", False):
            waiting.append("first GPS fix")
        if not astro_status.get("time_synced", False):
            waiting.append("time/date sync")
        if not waiting and not astro_ready:
            waiting.append("astronomic runtime")
        message = f"Waiting for {' and '.join(waiting)}."

    state.data["vision_status"] = vision_status
    state.data["astronomic_status"] = astro_status
    state.data["arduino_status"] = {
        "available": bool(astro_runtime is not None),
        "system_state": arduino_status.get("system_state", "UNKNOWN"),
        "calibration_done": bool(
            runtime_orchestrator.hw_calibrated
            or arduino_status.get("calibration_done", False)
        ),
        "last_response": arduino_status.get("last_response", ""),
        "last_debug_line": arduino_status.get("last_debug_line", ""),
        "motor_state": arduino_status.get(
            "motor_state", {"RA": "UNKNOWN", "DEC": "UNKNOWN", "FOC": "UNKNOWN"}
        ),
    }
    state.data["system"] = {
        "hardware_calibrated": hardware_calibrated,
        "vision_ready": bool(vision_status["ready"]),
        "astronomic_ready": astro_ready,
        "controls_ready": controls_ready,
        "catalog_ready": any(len(items) for items in state.catalog.values()),
        "message": message,
    }
    return controls_ready


def compute_catalog_hash(catalog):
    try:
        return json.dumps(catalog, sort_keys=True, ensure_ascii=True)
    except Exception:
        return str(catalog)


async def refresh_catalog_if_due(force=False):
    global catalog_refresh_deadline, catalog_last_hash
    now = time.monotonic()
    if not force and now < catalog_refresh_deadline:
        return

    if astro_runtime is None:
        if state.catalog != CATALOG:
            state.catalog = copy.deepcopy(CATALOG)
            catalog_last_hash = compute_catalog_hash(state.catalog)
            await broadcast_catalog()
        catalog_refresh_deadline = now + 300.0
        return

    try:
        next_catalog, delay = astro_runtime.compute_visible_catalog(
            timestamp_ms=state.data["timestamp"]
        )
    except Exception as exc:
        print(f"Catalog refresh failed: {exc}")
        catalog_refresh_deadline = now + 30.0
        return

    state.catalog = next_catalog
    current_target = state.data["mission"].get("target", "STANDBY")
    if current_target and current_target != "STANDBY":
        visible_ids = {
            str(entry.get("id"))
            for entries in state.catalog.values()
            for entry in entries
            if entry.get("id") is not None
        }
        if str(current_target) not in visible_ids:
            state.data["mission"]["target"] = "STANDBY"
            if state.data["mission"].get("action") == "GOTO":
                state.data["mission"]["action"] = "--"

    next_hash = compute_catalog_hash(next_catalog)
    if next_hash != catalog_last_hash:
        catalog_last_hash = next_hash
        await broadcast_catalog()
    catalog_refresh_deadline = now + max(5.0, float(delay or 60.0))


async def handle_photo_capture_request(source_ws=None):
    target_id = state.data["mission"].get("target", "STANDBY")
    directory, output_path = build_capture_paths(target_id, "photo")
    directory.mkdir(parents=True, exist_ok=True)
    ok, error_msg = cam.capture_photo(output_path)
    if not ok:
        state.data["capture"]["last_error"] = error_msg
        if source_ws is not None:
            await safe_send(source_ws, {"type": "TOAST", "msg": error_msg})
        return False

    state.data["capture"]["last_photo_at"] = utc_now_iso()
    state.data["capture"]["last_photo_path"] = str(output_path)
    state.data["capture"]["last_error"] = ""
    state.set_temporary_action("PHOTO", 3)
    return True


async def handle_video_capture_request(duration, source_ws=None):
    target_id = state.data["mission"].get("target", "STANDBY")
    directory, output_path = build_capture_paths(target_id, "video")
    directory.mkdir(parents=True, exist_ok=True)
    ok, error_msg = cam.start_video_recording(output_path, duration_seconds=duration)
    if not ok:
        state.data["capture"]["last_error"] = error_msg
        if source_ws is not None:
            await safe_send(source_ws, {"type": "TOAST", "msg": error_msg})
        return False

    state.data["capture"]["video_duration"] = duration
    state.data["capture"]["last_video_duration"] = duration
    state.data["capture"]["last_video_request_at"] = utc_now_iso()
    state.data["capture"]["last_video_path"] = str(output_path)
    state.data["capture"]["recording"] = True
    state.data["capture"]["last_error"] = ""
    state.set_temporary_action("VIDEO", duration)
    return True


async def execute_command(pkt, source_ws=None):
    global catalog_refresh_deadline
    ptype = pkt.get("type")
    should_broadcast = False

    if not is_command_allowed_during_init(ptype):
        if not refresh_system_status():
            if source_ws is not None:
                await safe_send(
                    source_ws,
                    {
                        "type": "TOAST",
                        "msg": state.data["system"]["message"],
                    },
                )
            return

    if ptype == "JOYSTICK":
        data = pkt.get("data", {})
        state.data["joystick"] = {
            "x": float(data.get("x", 0.0)),
            "y": float(data.get("y", 0.0)),
        }
        should_broadcast = True

    elif ptype == "EMERGENCY_STOP":
        state.data["mission"]["action"] = "EMERGENCY_STOP"
        state.action_expires_at = None
        await trigger_emergency_stop()
        return

    elif ptype == "MISSION":
        requested_target = pkt.get("target", "STANDBY")
        if requested_target != "STANDBY":
            visible_ids = {
                str(entry.get("id"))
                for entries in state.catalog.values()
                for entry in entries
                if entry.get("id") is not None
            }
            if str(requested_target) not in visible_ids:
                if source_ws is not None:
                    await safe_send(
                        source_ws,
                        {
                            "type": "TOAST",
                            "msg": "Selected mission is not currently visible.",
                        },
                    )
                return

        state.data["mission"]["target"] = requested_target
        state.data["mission"]["action"] = pkt.get("action", "GOTO")
        state.action_expires_at = None
        if state.data["mission"]["action"] not in {"PHOTO", "VIDEO"}:
            state.last_action_before_temp = state.data["mission"]["action"]
        should_broadcast = True

    elif ptype == "COMMAND":
        action = pkt.get("action", "--")
        if action == "PHOTO":
            should_broadcast = await handle_photo_capture_request(source_ws=source_ws)
        elif action == "VIDEO":
            duration = clamp(int(pkt.get("duration", 10)), 1, 3600)
            should_broadcast = await handle_video_capture_request(
                duration=duration, source_ws=source_ws
            )
        else:
            state.data["mission"]["action"] = action
            state.action_expires_at = None
            state.last_action_before_temp = action
            should_broadcast = True

    elif ptype == "GPS_UPDATE":
        data = pkt.get("data", pkt)
        state.data["gps"] = {
            "lat": float(data.get("lat", 0.0) or 0.0),
            "lon": float(data.get("lon", 0.0) or 0.0),
            "alt": float(data.get("alt", 0.0) or 0.0),
        }
        if astro_runtime is not None:
            astro_runtime.set_location(
                state.data["gps"]["lat"],
                state.data["gps"]["lon"],
                state.data["gps"]["alt"],
            )
        catalog_refresh_deadline = 0.0
        should_broadcast = True

    elif ptype in {"SYNC_CLOCK", "TIMESTAMP", "DATETIME"}:
        timestamp_ms = pkt.get("timestamp")
        if timestamp_ms is None:
            timestamp_ms = pkt.get("data")
        datetime_value = pkt.get("datetime")
        if datetime_value is None and ptype == "DATETIME":
            datetime_value = pkt.get("data")
        if timestamp_ms is not None:
            try:
                timestamp_ms = int(float(timestamp_ms))
            except (TypeError, ValueError):
                timestamp_ms = None
        state.sync_clock(
            timestamp_ms=timestamp_ms,
            datetime_value=datetime_value,
            explicit=True,
        )
        if astro_runtime is not None:
            astro_runtime.set_time(timestamp_ms=timestamp_ms, datetime_value=datetime_value)
        catalog_refresh_deadline = 0.0
        should_broadcast = True

    elif ptype == "CAMERA_SETTINGS":
        camera = state.data["camera"]
        payload = pkt.get("data", {})
        if "exposure" in payload:
            camera["exposure"] = clamp(int(payload["exposure"]), 1, 10000000)
        if "gain" in payload:
            camera["gain"] = clamp(int(payload["gain"]), 0, 600)
        if payload.get("type") in {"auto", "deep_sky", "planet"}:
            camera["type"] = payload["type"]
        cam.apply_camera_settings(camera)
        should_broadcast = True

    elif ptype == "MOTOR_SETTINGS":
        motor = state.data["motor"]
        payload = pkt.get("data", {})
        if "tracking_with_camera" in payload:
            motor["tracking_with_camera"] = bool(payload["tracking_with_camera"])
        if "compensate_earth_rotation" in payload:
            motor["compensate_earth_rotation"] = bool(
                payload["compensate_earth_rotation"]
            )
        should_broadcast = True

    elif ptype == "FOCUS_SETTINGS":
        focus = state.data["focus"]
        payload = pkt.get("data", {})
        if payload.get("mode") in {"auto", "manual"}:
            focus["mode"] = payload["mode"]
        # Tick-domain value has priority when provided.
        max_ticks = vision_runtime.TOTAL_FOCUS_STEPS
        if "manual_ticks" in payload:
            try:
                manual_ticks = _clamp_float(payload["manual_ticks"], 0.0, max_ticks)
            except Exception:
                manual_ticks = float(focus.get("manual_ticks", 0.0))
            focus["manual_ticks"] = int(round(manual_ticks))
            focus["manual_value"] = clamp(int(round((manual_ticks / max_ticks) * 100.0)), 0, 100)
        if "manual_value" in payload:
            focus["manual_value"] = clamp(int(payload["manual_value"]), 0, 100)
            focus["manual_ticks"] = int(round((float(focus["manual_value"]) / 100.0) * max_ticks))
        cam.apply_focus_settings(focus)
        vision_runtime.update_focus_settings(focus["mode"], focus["manual_value"])
        if astro_runtime is not None:
            astro_runtime.set_focus_info(
                mode=focus["mode"],
                manual_value=focus["manual_value"],
                best_step=vision_runtime.best_focus_step,
                current_step=vision_runtime.current_focus_step,
                manual_ticks=focus.get("manual_ticks"),
            )
        should_broadcast = True

    elif ptype == "AUTOFOCUS_RECALIBRATE":
        if not runtime_orchestrator.hw_calibrated:
            if source_ws is not None:
                await safe_send(
                    source_ws,
                    {"type": "TOAST", "msg": "Hardware calibration is not complete yet."},
                )
            return
        vision_runtime.start_focus_calibration()
        runtime_orchestrator.focus_calibration_started = True
        runtime_orchestrator.focus_calibrated = False
        should_broadcast = True

    elif ptype == "VIDEO_REQUEST":
        duration = clamp(int(pkt.get("duration", 10)), 1, 3600)
        should_broadcast = await handle_video_capture_request(
            duration=duration, source_ws=source_ws
        )

    elif ptype == "PHOTO_REQUEST":
        should_broadcast = await handle_photo_capture_request(source_ws=source_ws)

    elif ptype == "RUNTIME_MODES":
        payload = pkt.get("data", {})
        modes = state.data.get("runtime_modes", {})
        if "simulate_arduino" in payload:
            modes["simulate_arduino"] = bool(payload["simulate_arduino"])
            if astro_runtime is not None:
                try:
                    astro_runtime.motor_link.simulate = bool(payload["simulate_arduino"])
                except Exception:
                    pass
        if "camera_mode" in payload:
            requested = str(payload["camera_mode"]).lower()
            if requested in {"hardware", "sim"}:
                modes["camera_mode"] = cam.mode
                if source_ws is not None:
                    await safe_send(
                        source_ws,
                        {
                            "type": "TOAST",
                            "msg": "Camera mode changes require a service restart.",
                        },
                    )
        state.data["runtime_modes"] = modes
        should_broadcast = True

    if should_broadcast:
        await broadcast_telemetry()


async def handle_request_response(ws, pkt):
    if not state.can_admin(ws):
        return

    request_id = pkt.get("req_id")
    approved = bool(pkt.get("approved"))
    request = state.pending_requests.pop(request_id, None)

    if request is None:
        await safe_send(ws, {"type": "TOAST", "msg": "Request no longer available."})
        return

    requester = request["ws"]
    if requester not in state.clients:
        await broadcast_system_info()
        return

    if not approved:
        await safe_send(requester, {"type": "TOAST", "msg": "Request denied."})
        await broadcast_system_info()
        return

    kind = request["kind"]
    if kind == "CMD_REQUEST":
        if request["sub_kind"] == "FULL":
            state.set_commander(requester, state.client_name(requester))
            await safe_send(
                requester, {"type": "TOAST", "msg": "You are now commander."}
            )
        else:
            if requester not in state.commander_queue:
                state.commander_queue.append(requester)
            await safe_send(
                requester,
                {"type": "TOAST", "msg": "Added to the commander queue."},
            )
        await broadcast_system_info()
        return

    if kind == "ACTION_REQUEST":
        await execute_command(request["payload"], source_ws=requester)
        await safe_send(requester, {"type": "TOAST", "msg": "Action approved."})
        await broadcast_system_info()
        return

    if kind == "JOYSTICK_REQUEST":
        await grant_joystick(requester, request["duration"])
        await safe_send(
            requester,
            {
                "type": "TOAST",
                "msg": "Joystick privilege approved. Open it with the joystick button.",
            },
        )
        return

    if kind == "JOYSTICK_QUEUE_REQUEST":
        if state.queue_joystick(requester, request["duration"]):
            await safe_send(
                requester,
                {"type": "TOAST", "msg": "Added to the joystick queue."},
            )
        else:
            await safe_send(
                requester, {"type": "TOAST", "msg": "Already queued for joystick."}
            )
        await broadcast_system_info()


async def handle_websocket(request):
    ws = web.WebSocketResponse(heartbeat=20)
    await ws.prepare(request)

    if state.emergency_stop:
        await safe_send(ws, {"type": "EMERGENCY_STOP"})
        await ws.close()
        return ws

    state.register(ws, request.remote)
    if state.commander_ws is None:
        state.set_commander(ws, "FirstUser")

    await send_full_state(ws)
    await broadcast_system_info()
    await broadcast_telemetry()

    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue

            try:
                pkt = json.loads(msg.data)
            except json.JSONDecodeError:
                continue

            record_exchange("rx", pkt, ws=ws, delivered=True)
            ptype = pkt.get("type")

            if ptype == "LOGIN_BRIDGE":
                if pkt.get("pin") != BRIDGE_PIN:
                    await safe_send(ws, {"type": "LOGIN_FAIL"})
                    continue

                state.clients[ws]["role"] = "BRIDGE"
                state.clients[ws]["name"] = "BRIDGE"
                if state.commander_ws == ws:
                    state.commander_ws = None
                    state.promote_next_commander()
                await safe_send(ws, {"type": "LOGIN_SUCCESS", "data": state.data})
                await send_full_state(ws)
                await broadcast_system_info()
                continue

            if ptype == "SET_NAME":
                state.set_client_name(ws, pkt.get("name", "Anon"))
                await broadcast_system_info()
                continue

            if ptype == "REQUEST_COMMANDER":
                if state.is_bridge(ws):
                    continue
                name = pkt.get("name", state.client_name(ws) or "Anon")
                state.set_client_name(ws, name)
                if state.is_commander(ws):
                    await safe_send(
                        ws, {"type": "TOAST", "msg": "You are already commander."}
                    )
                    continue
                request_mode = pkt.get("req_mode", "FULL").upper()
                if request_mode not in {"FULL", "QUEUE"}:
                    request_mode = "FULL"

                if (
                    request_mode == "QUEUE"
                    and ws in state.commander_queue
                    or state.has_pending_request(ws, {"CMD_REQUEST"})
                ):
                    await safe_send(
                        ws,
                        {
                            "type": "TOAST",
                            "msg": "You already have a commander request in progress.",
                        },
                    )
                    continue

                if state.commander_ws is None:
                    state.set_commander(ws, name)
                    await safe_send(ws, {"type": "TOAST", "msg": "You are now commander."})
                    await broadcast_system_info()
                    continue

                request_id = state.next_request_id()
                state.pending_requests[request_id] = {
                    "kind": "CMD_REQUEST",
                    "sub_kind": request_mode,
                    "ws": ws,
                }
                await notify_request(request_id)
                await safe_send(ws, {"type": "TOAST", "msg": "Commander request sent."})
                continue

            if ptype == "SET_COMMANDER":
                if not state.is_bridge(ws):
                    continue
                try:
                    target_id = int(pkt.get("client_id"))
                except (TypeError, ValueError):
                    await safe_send(ws, {"type": "TOAST", "msg": "Invalid commander selection."})
                    continue
                target_ws = state.client_ws_by_id(target_id)
                if target_ws is None or state.is_bridge(target_ws):
                    await safe_send(ws, {"type": "TOAST", "msg": "Commander target not available."})
                    continue
                state.set_commander(target_ws, state.client_name(target_ws))
                await safe_send(
                    target_ws, {"type": "TOAST", "msg": "You are now commander."}
                )
                await broadcast_system_info()
                continue

            if ptype == "REQUEST_ACTION":
                request_id = state.next_request_id()
                state.pending_requests[request_id] = {
                    "kind": "ACTION_REQUEST",
                    "ws": ws,
                    "desc": pkt.get("desc", "Action request"),
                    "payload": pkt.get("payload", {}),
                }
                await notify_request(request_id)
                await safe_send(ws, {"type": "TOAST", "msg": "Action request sent."})
                continue

            if ptype == "REQUEST_JOYSTICK":
                duration = clamp(int(pkt.get("duration", 30)), 10, 3600)

                if state.is_commander(ws):
                    await grant_joystick(ws, reason="commander_override")
                    continue

                if state.joystick_owner == ws:
                    await safe_send(
                        ws,
                        {
                            "type": "JOYSTICK_GRANTED",
                            "duration": duration,
                            "remaining": state.joystick_remaining(),
                        },
                    )
                    continue

                if state.has_pending_request(
                    ws, {"JOYSTICK_REQUEST", "JOYSTICK_QUEUE_REQUEST"}
                ):
                    await safe_send(
                        ws,
                        {
                            "type": "TOAST",
                            "msg": "Your joystick request is already awaiting approval.",
                        },
                    )
                    continue

                if state.has_pending_joystick_request():
                    await safe_send(
                        ws,
                        {
                            "type": "TOAST",
                            "msg": "Another joystick request is already awaiting approval.",
                        },
                    )
                    continue

                request_kind = (
                    "JOYSTICK_REQUEST"
                    if state.joystick_owner is None
                    else "JOYSTICK_QUEUE_REQUEST"
                )
                request_id = state.next_request_id()
                state.pending_requests[request_id] = {
                    "kind": request_kind,
                    "ws": ws,
                    "duration": duration,
                }
                await notify_request(request_id)
                await safe_send(ws, {"type": "JOYSTICK_PENDING", "kind": request_kind})
                continue

            if ptype == "CMD_RESPONSE":
                await handle_request_response(ws, pkt)
                continue

            if ptype == "TOGGLE_MUTE":
                if state.can_admin(ws):
                    state.mute_notifications = bool(pkt.get("mute", False))
                    await broadcast_system_info()
                continue

            if ptype == "JOYSTICK_QUEUE_MANAGE":
                await manage_joystick_queue(ws, pkt)
                continue

            if ptype == "JOYSTICK_OWNER_TIME":
                await adjust_joystick_owner_duration(ws, pkt)
                continue

            if ptype == "REVOKE_JOYSTICK":
                if state.can_admin(ws):
                    await revoke_joystick(reason="revoked_by_admin", advance_queue=True)
                continue

            if ptype == "RELEASE_JOYSTICK":
                if ws == state.joystick_owner or state.can_admin(ws):
                    await revoke_joystick(reason="released", advance_queue=True)
                continue

            if ptype == "JOYSTICK":
                if ws == state.joystick_owner:
                    await execute_command(pkt, source_ws=ws)
                continue

            if state.can_admin(ws) or state.is_commander(ws):
                await execute_command(pkt, source_ws=ws)

    finally:
        lost_owner = ws == state.joystick_owner
        state.unregister(ws)
        if lost_owner:
            await grant_next_joystick()
        await broadcast_system_info()
        await broadcast_telemetry()

    return ws


async def video_feed(request):
    response = web.StreamResponse(
        status=200,
        reason="OK",
        headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
            "Access-Control-Allow-Origin": "*",
        },
    )
    await response.prepare(request)

    try:
        while True:
            frame = cam.get_frame()
            if frame:
                await response.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8")
                    + frame
                    + b"\r\n"
                )
            await asyncio.sleep(0.04)
    except Exception:
        pass

    return response


async def page_root(request):
    if state.emergency_stop:
        return web.FileResponse("./public/emergency.html")
    return web.FileResponse("./public/remote.html")


async def page_remote(request):
    if state.emergency_stop:
        raise web.HTTPFound("/")
    return web.FileResponse("./public/remote.html")


async def page_bridge(request):
    if state.emergency_stop:
        raise web.HTTPFound("/")
    return web.FileResponse("./public/bridge.html")


async def debug_state(request):
    snapshot = {
        "ts": utc_now_iso(),
        "data": state.data,
        "catalog": state.catalog,
        "clients": [state.client_meta(ws) for ws in state.clients.keys()],
        "system": state.serialize_system_info(state.commander_ws)
        if state.commander_ws in state.clients
        else {"type": "SYS_INFO", "users": []},
    }
    return web.json_response(snapshot)


async def debug_exchanges(request):
    try:
        limit = int(request.query.get("limit", "200"))
    except ValueError:
        limit = 200
    return web.json_response({"events": state.exchange_snapshot(limit=limit)})


async def handle_runtime_orchestration(vision_payload):
    if astro_runtime is None:
        return

    link = astro_runtime.motor_link
    events = link.drain_events(limit=200)

    for event in events:
        if event.get("kind") != "ack":
            continue
        code = event.get("code")
        action = event.get("action")
        runtime_orchestrator.last_handshake_code = code

        if action == 3:
            if code == 1:
                runtime_orchestrator.hw_calibration_accepted = True
            elif code == 2:
                runtime_orchestrator.hw_calibrated = True
            elif code == 0:
                runtime_orchestrator.hw_calibration_accepted = False
                runtime_orchestrator.hw_next_retry_at = time.monotonic() + 1.0

    # Hardware calibration handshake with graceful retry.
    now = time.monotonic()
    should_send_calib = False
    if not runtime_orchestrator.hw_calibration_requested:
        should_send_calib = True
    elif (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and now >= runtime_orchestrator.hw_next_retry_at
        and runtime_orchestrator.hw_calibration_retries > 0
    ):
        should_send_calib = True

    if should_send_calib and not runtime_orchestrator.hw_calibrated:
        sent = link.send_command(3)
        runtime_orchestrator.hw_calibration_requested = bool(sent)
        runtime_orchestrator.hw_calibration_requested_at = now
        if sent:
            runtime_orchestrator.hw_calibration_retries += 1
            runtime_orchestrator.hw_next_retry_at = now + CALIBRATION_COMMAND_TIMEOUT_S
            print(
                f"Startup calibration request sent (3), try #{runtime_orchestrator.hw_calibration_retries}."
            )
        else:
            runtime_orchestrator.hw_next_retry_at = now + 1.0
            print("Failed to send startup calibration command (3) to Arduino.")

    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and runtime_orchestrator.hw_calibration_accepted
    ):
        elapsed = time.monotonic() - runtime_orchestrator.hw_calibration_requested_at
        if elapsed > CALIBRATION_COMMAND_TIMEOUT_S:
            # Do not force ERROR here; keep system alive and let UI remain accessible.
            print("Warning: hardware calibration timeout waiting for done code 2.")
            runtime_orchestrator.hw_calibration_accepted = False
            runtime_orchestrator.hw_next_retry_at = time.monotonic() + 1.0

    # Backward-compatible fallback when firmware prints state but does not emit "2".
    last_dbg = link.status().get("last_debug_line", "")
    if (
        runtime_orchestrator.hw_calibration_accepted
        and not runtime_orchestrator.hw_calibrated
        and "mount=READY" in str(last_dbg)
    ):
        runtime_orchestrator.hw_calibrated = True

    if runtime_orchestrator.hw_calibrated and not runtime_orchestrator.focus_calibration_started:
        vision_runtime.start_focus_calibration()
        runtime_orchestrator.focus_calibration_started = True

    # Run autofocus handshake against Arduino command acks.
    if runtime_orchestrator.focus_calibration_started:
        vision_runtime.handle_arduino_events(events, link)
        vision_runtime.tick_autofocus(link)

    if runtime_orchestrator.focus_calibration_started and vision_runtime.to_payload().get("ready", False):
        runtime_orchestrator.focus_calibrated = True

    tracking_enabled = bool(
        runtime_orchestrator.hw_calibrated
        and runtime_orchestrator.focus_calibrated
        and astro_runtime.ready
    )
    astro_runtime.set_command_gates(
        hardware_ready=runtime_orchestrator.hw_calibrated,
        focus_ready=runtime_orchestrator.focus_calibrated,
        tracking_enabled=tracking_enabled,
    )


async def background_state_loop(app):
    last_telemetry_push = 0.0
    last_system_info_push = 0.0
    while True:
        state.update_clock()
        state.clear_temporary_action_if_needed()

        frame = cam.get_raw_frame_copy()
        vision_runtime.update_from_frame(frame)
        # Orchestration can change autofocus state (commands/acks), so we refresh
        # payload after running it.
        await handle_runtime_orchestration(vision_runtime.to_payload())
        vision_payload = vision_runtime.to_payload()
        state.data["vision_status"] = vision_payload

        if astro_runtime is not None:
            try:
                astro_runtime.set_tracking_error(
                    dx=vision_payload.get("dx", 0.0),
                    dy=vision_payload.get("dy", 0.0),
                    frame_width=vision_payload.get("frame_width", VIDEO_WIDTH),
                    frame_height=vision_payload.get("frame_height", VIDEO_HEIGHT),
                )
                astro_runtime.set_focus_info(
                    mode=vision_payload.get("mode", "auto"),
                    manual_value=vision_payload.get("manual_value", 50),
                    best_step=vision_payload.get("best_step", 0.0),
                    current_step=vision_payload.get("current_step", 0.0),
                    manual_ticks=state.data.get("focus", {}).get("manual_ticks"),
                )
                astro_runtime.tick(state.data)
            except Exception as exc:
                print(f"Astronomic tick failed: {exc}")

        recording_status = cam.recording_status()
        state.data["capture"]["recording"] = bool(recording_status["active"])
        if recording_status["last_error"]:
            state.data["capture"]["last_error"] = recording_status["last_error"]
        if recording_status["last_path"]:
            state.data["capture"]["last_video_path"] = recording_status["last_path"]

        refresh_system_status()

        if (
            state.joystick_owner is not None
            and state.joystick_expires_at is not None
            and time.time() >= state.joystick_expires_at
        ):
            await revoke_joystick(reason="expired", advance_queue=True)
        await refresh_catalog_if_due(force=False)

        now = time.monotonic()
        if (now - last_telemetry_push) >= 0.1:
            await broadcast_telemetry()
            last_telemetry_push = now
        if (now - last_system_info_push) >= 0.5:
            await broadcast_system_info()
            last_system_info_push = now

        await asyncio.sleep(BACKGROUND_LOOP_INTERVAL_S)


async def start_background_tasks(app):
    global astro_runtime, catalog_refresh_deadline, catalog_last_hash
    runtime_orchestrator.reset()
    CAPTURE_ROOT.mkdir(parents=True, exist_ok=True)
    EXCHANGE_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    EXCHANGE_LOG_FILE.touch(exist_ok=True)

    env_sim_arduino = os.environ.get("STARTRACK_SIM_ARDUINO", "").strip().lower()
    if env_sim_arduino in {"1", "true", "yes", "on"}:
        simulate_arduino = True
    elif env_sim_arduino in {"0", "false", "no", "off"}:
        simulate_arduino = False
    else:
        simulate_arduino = cam.mode == "sim"

    state.data["runtime_modes"] = {
        "camera_mode": cam.mode,
        "simulate_arduino": simulate_arduino,
    }

    requested_port = os.environ.get("STARTRACK_ARDUINO_PORT", "auto").strip()
    if requested_port.lower() == "auto":
        arduino_port = detect_arduino_port("COM6" if os.name == "nt" else "/dev/ttyUSB0")
    else:
        arduino_port = requested_port
    arduino_baud = int(os.environ.get("STARTRACK_ARDUINO_BAUD", "115200"))
    state.data["runtime_modes"]["arduino_port"] = arduino_port

    if AstronomicOperator is not None:
        try:
            astro_runtime = AstronomicOperator(
                base_catalog=CATALOG,
                latitude=state.data["gps"]["lat"],
                longitude=state.data["gps"]["lon"],
                altitude_m=state.data["gps"]["alt"],
                data_root=str(PROJECT_ROOT),
                simulate_arduino=simulate_arduino,
                arduino_port=arduino_port,
                arduino_baudrate=arduino_baud,
            )
        except Exception as exc:
            print(f"Astronomic runtime startup failed: {exc}")
            astro_runtime = None
    else:
        astro_runtime = None

    focus_cfg = state.data.get("focus", {})
    if "manual_ticks" not in focus_cfg:
        focus_cfg["manual_ticks"] = int(
            round((float(focus_cfg.get("manual_value", 50)) / 100.0) * vision_runtime.TOTAL_FOCUS_STEPS)
        )
    focus_cfg["manual_ticks"] = int(
        round(_clamp_float(focus_cfg.get("manual_ticks", 0), 0.0, vision_runtime.TOTAL_FOCUS_STEPS))
    )
    focus_cfg["manual_value"] = clamp(
        int(round((float(focus_cfg["manual_ticks"]) / vision_runtime.TOTAL_FOCUS_STEPS) * 100.0)),
        0,
        100,
    )
    state.data["focus"] = focus_cfg

    vision_runtime.update_focus_settings(
        state.data["focus"]["mode"], state.data["focus"]["manual_value"]
    )
    if astro_runtime is not None:
        astro_runtime.set_command_gates(
            hardware_ready=False,
            focus_ready=False,
            tracking_enabled=False,
        )
        astro_runtime.set_focus_info(
            mode=state.data["focus"]["mode"],
            manual_value=state.data["focus"]["manual_value"],
            best_step=vision_runtime.best_focus_step,
            current_step=vision_runtime.current_focus_step,
            manual_ticks=state.data["focus"].get("manual_ticks"),
        )
    if astro_runtime is not None and not astro_runtime.ready:
        state.catalog = {tab: [] for tab in CATALOG}
    refresh_system_status()
    catalog_last_hash = compute_catalog_hash(state.catalog)
    catalog_refresh_deadline = 0.0
    await refresh_catalog_if_due(force=True)
    app["state_loop"] = asyncio.create_task(background_state_loop(app))


async def cleanup_background_tasks(app):
    app["state_loop"].cancel()
    try:
        await app["state_loop"]
    except asyncio.CancelledError:
        pass
    if astro_runtime is not None:
        try:
            astro_runtime.close()
        except Exception:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="hardware")
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("STARTRACK_PORT", "8443")),
    )
    args = parser.parse_args()

    cam = CameraManager(args.mode)
    cam.start()

    app = web.Application()
    app.router.add_get("/", page_root)
    app.router.add_get("/remote", page_remote)
    app.router.add_get("/bridge", page_bridge)
    app.router.add_get("/stream", video_feed)
    app.router.add_get("/debug/state", debug_state)
    app.router.add_get("/debug/exchanges", debug_exchanges)
    app.router.add_get("/ws", handle_websocket)
    app.router.add_static("/static", "./public/static")
    app.on_startup.append(start_background_tasks)
    app.on_cleanup.append(cleanup_background_tasks)

    ssl_ctx = None
    if os.path.exists(SSL_CERT) and os.path.exists(SSL_KEY):
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(SSL_CERT, SSL_KEY)
        print("--- STARTRACK READY (HTTPS) ---")
    else:
        print("--- STARTRACK READY (HTTP fallback: cert.pem/key.pem missing) ---")
    web.run_app(app, port=args.port, ssl_context=ssl_ctx)
