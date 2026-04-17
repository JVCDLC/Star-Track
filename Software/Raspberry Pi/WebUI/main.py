"""
StarTrack WebUI backend runtime.

This module is the integration hub for the full telescope stack:

1) WebSocket + HTTP API layer (remote control, bridge panel, telemetry stream)
2) Camera acquisition and media capture orchestration
3) Vision coordinator state machine (autofocus sweeps + tracking offsets)
4) Astronomic operator bridge (target conversion + Arduino command flow)
5) Global initialization/runtime safety gating

Important maintenance note:
- Keep this file behaviorally stable. Most sections are tightly coupled through
  shared state (`state.data`) and timed background loops.
- When changing logic, always validate startup handshake, calibration gates,
  and command permissions together.
"""

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

# -----------------------------------------------------------------------------
# Path setup and optional astronomy runtime import
# -----------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parents[3]
ASTRO_SRC_DIR = PROJECT_ROOT / "Software" / "Raspberry Pi" / "Astronomic Operator"
if str(ASTRO_SRC_DIR) not in sys.path:
    sys.path.append(str(ASTRO_SRC_DIR))

try:
    from astronomic_operator import AstronomicOperator
except Exception as exc:
    AstronomicOperator = None
    print(f"Astronomic operator import failed: {exc}")


# -----------------------------------------------------------------------------
# Global configuration constants
# -----------------------------------------------------------------------------
SSL_CERT = "cert.pem"
SSL_KEY = "key.pem"
ASI_LIB = "/usr/lib/libASICamera2.so"
VIDEO_WIDTH = 1304
VIDEO_HEIGHT = 976
JPEG_QUALITY = 60
PNG_COMPRESSION = 0
BRIDGE_PIN = "1234"
CAPTURE_ROOT = Path("startrack_img")
VISION_INIT_SECONDS = 8.0
EXCHANGE_LOG_FILE = Path(__file__).resolve().parent / "exchange.log"
SERIAL_EXCHANGE_LOG_FILE = Path(__file__).resolve().parent / "serial_exchange.log"
VISION_DEBUG_LOG_FILE = Path(__file__).resolve().parent / "vision_focus_debug.log"
EXCHANGE_LOG_LIMIT = 2000
SERIAL_EXCHANGE_LOG_LIMIT = 6000
VISION_DEBUG_EVENT_LIMIT = 20000
BACKGROUND_LOOP_INTERVAL_S = 0.05
CALIBRATION_COMMAND_TIMEOUT_S = 180.0

# -----------------------------------------------------------------------------
# Mission catalog used by both UI and astronomy runtime
# -----------------------------------------------------------------------------
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

SOLAR_TARGET_IDS = {
    str(item.get("id", "")).strip().upper()
    for item in CATALOG.get("tab-solar", [])
    if str(item.get("id", "")).strip()
}
SOLAR_TARGET_NAMES = {
    str(item.get("name", "")).strip().upper()
    for item in CATALOG.get("tab-solar", [])
    if str(item.get("name", "")).strip()
}


# -----------------------------------------------------------------------------
# Utility helpers
# -----------------------------------------------------------------------------
def utc_now_iso():
    """
    Return a compact UTC ISO timestamp string used in telemetry and log events.
    """
    return datetime.utcnow().replace(microsecond=0).isoformat()


def clamp(value, low, high):
    """
    Clamp an integer-like value to [low, high] bounds.
    """
    return max(low, min(high, value))


def _clamp_float(value, low, high):
    """
    Clamp a float-convertible value to [low, high] with safe fallback on parse errors.
    """
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = low
    return max(float(low), min(float(high), value))


def parse_datetime_to_ms(value):
    """
    Parse ISO-like datetime text into epoch milliseconds (or None when invalid).
    """
    if not value:
        return None
    try:
        normalized = value.replace("Z", "+00:00")
        return int(datetime.fromisoformat(normalized).timestamp() * 1000)
    except (TypeError, ValueError):
        return None


def sanitize_fs_name(value, fallback="UNKNOWN"):
    """
    Sanitize dynamic text so it is safe to use as a filesystem folder/file segment.
    """
    text = str(value or "").strip()
    if not text:
        text = fallback
    text = re.sub(r"[\\/:*?\"<>|]+", "_", text)
    text = re.sub(r"\s+", "_", text)
    return text[:64] or fallback


def mission_target_is_solar(target_id):
    """
    Return True when a mission target belongs to the solar catalog group.
    """
    target = str(target_id or "").strip().upper()
    if not target or target == "STANDBY":
        return False
    return target in SOLAR_TARGET_IDS or target in SOLAR_TARGET_NAMES


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
        """
        Score serial port metadata so Arduino-like devices are prioritized.
        """
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


# -----------------------------------------------------------------------------
# Shared application state model
# -----------------------------------------------------------------------------
class SystemState:
    """
    In-memory shared backend state.

    This object is the single source of truth for:
    - websocket clients/roles/queues,
    - mission + telemetry payload mirrored to UIs,
    - runtime coordination values used by background tasks.
    """
    def __init__(self):
        """
        Initialize object state used by cross-module orchestration and runtime safety gates.
        """
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
        # `catalog` is the visible mission subset, independent from static CATALOG.
        self.catalog = copy.deepcopy(CATALOG)
        self.exchange_log = deque(maxlen=EXCHANGE_LOG_LIMIT)

        # Payload exposed to frontend as `TELEMETRY.data`.
        self.data = {
            # Core navigation/control context.
            "gps": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "joystick": {"x": 0.0, "y": 0.0},
            "mission": {"target": "STANDBY", "action": "--"},
            "timestamp": now_ms,
            "datetime": utc_now_iso(),
            # Runtime configurable camera and motor behavior.
            "camera": {"exposure": 30000, "gain": 150, "type": "auto"},
            "motor": {
                "tracking_with_camera": True,
                "compensate_earth_rotation": True,
            },
            # Backend runtime mode information (sim/hardware).
            "runtime_modes": {
                "camera_mode": "hardware",
                "simulate_arduino": False,
            },
            # Focus control + capture + subsystem status channels.
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
        """
        Allocate a monotonically increasing request ID used to track approval workflows end-to-end.
        """
        self.request_seq += 1
        return self.request_seq

    def register(self, ws, ip):
        """
        Register a websocket client and seed identity/role metadata consumed by queue and permission logic.
        """
        self.client_seq += 1
        self.clients[ws] = {
            "id": self.client_seq,
            "name": "Anon",
            "role": "OBSERVER",
            "ip": ip or "unknown",
        }

    def unregister(self, ws):
        """
        Detach a websocket client and cleanup commander/joystick/pending-request ownership references.
        """
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
        """
        Cleanup ephemeral state to prevent stale references and cross-client side effects.
        """
        stale = []
        for req_id, request in self.pending_requests.items():
            if request["ws"] == ws:
                stale.append(req_id)
        for req_id in stale:
            del self.pending_requests[req_id]

    def has_pending_request(self, ws, kinds=None):
        """
        Return guard conditions used to gate command execution and privilege-sensitive operations.
        """
        for request in self.pending_requests.values():
            if request["ws"] != ws:
                continue
            if kinds is None or request["kind"] in kinds:
                return True
        return False

    def remove_from_commander_queue(self, ws):
        """
        Cleanup ephemeral state to prevent stale references and cross-client side effects.
        """
        self.commander_queue = deque(
            queued_ws for queued_ws in self.commander_queue if queued_ws != ws
        )

    def remove_from_joystick_queue(self, ws):
        """
        Cleanup ephemeral state to prevent stale references and cross-client side effects.
        """
        self.joystick_queue = deque(
            entry for entry in self.joystick_queue if entry["ws"] != ws
        )

    def client_name(self, ws):
        """
        Resolve client identity/role metadata required by permission and queue workflows.
        """
        client = self.clients.get(ws)
        return client["name"] if client else "Unknown"

    def client_id(self, ws):
        """
        Resolve client identity/role metadata required by permission and queue workflows.
        """
        client = self.clients.get(ws)
        return client["id"] if client else None

    def client_ws_by_id(self, client_id):
        """
        Resolve client identity/role metadata required by permission and queue workflows.
        """
        for ws, client in self.clients.items():
            if client.get("id") == client_id:
                return ws
        return None

    def client_role(self, ws):
        """
        Resolve client identity/role metadata required by permission and queue workflows.
        """
        client = self.clients.get(ws)
        return client["role"] if client else "OBSERVER"

    def client_meta(self, ws):
        """
        Resolve client identity/role metadata required by permission and queue workflows.
        """
        return {
            "client_id": self.client_id(ws),
            "name": self.client_name(ws),
            "role": self.client_role(ws),
        }

    def is_commander(self, ws):
        """
        Return guard conditions used to gate command execution and privilege-sensitive operations.
        """
        return ws is not None and ws == self.commander_ws

    def is_bridge(self, ws):
        """
        Return guard conditions used to gate command execution and privilege-sensitive operations.
        """
        return self.client_role(ws) == "BRIDGE"

    def can_admin(self, ws):
        """
        Return guard conditions used to gate command execution and privilege-sensitive operations.
        """
        return self.is_commander(ws) or self.is_bridge(ws)

    def set_client_name(self, ws, name):
        """
        Apply a state mutation while preserving invariants relied upon by other runtime stages.
        """
        if ws in self.clients and name:
            self.clients[ws]["name"] = name[:32]

    def set_commander(self, ws, name=None):
        """
        Apply a state mutation while preserving invariants relied upon by other runtime stages.
        """
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
        """
        Promote the next eligible user to commander (queue-first, then first non-bridge fallback).
        """
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
        """
        Return guard conditions used to gate command execution and privilege-sensitive operations.
        """
        return any(
            request["kind"] in {"JOYSTICK_REQUEST", "JOYSTICK_QUEUE_REQUEST"}
            for request in self.pending_requests.values()
        )

    def queue_joystick(self, ws, duration):
        """
        Add a client to joystick queue with requested duration if not already queued.
        """
        if ws not in self.clients:
            return False
        if any(entry["ws"] == ws for entry in self.joystick_queue):
            return False
        self.joystick_queue.append({"ws": ws, "duration": duration})
        return True

    def pop_next_joystick(self):
        """
        Pop and return the next queued joystick entry that still belongs to a connected client.
        """
        while self.joystick_queue:
            entry = self.joystick_queue.popleft()
            if entry["ws"] in self.clients:
                return entry
        return None

    def serialize_joystick_queue(self):
        """
        Serialize internal state into stable transport payload structures for websocket clients.
        """
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
        """
        Reorder queue/prioritization data while preserving stable client mappings.
        """
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
        """
        Cleanup ephemeral state to prevent stale references and cross-client side effects.
        """
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
        """
        Adjust bounded runtime parameters and keep values within safe limits.
        """
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
        """
        Synchronize logical runtime clocks/state with external input values.
        """
        derived_ms = parse_datetime_to_ms(datetime_value)
        effective_ms = timestamp_ms or derived_ms or int(time.time() * 1000)
        self.clock_base_ms = int(effective_ms)
        self.clock_synced_at = time.monotonic()
        if explicit:
            self.clock_was_explicitly_synced = True
        self.update_clock()

    def update_clock(self):
        """
        Update internal state using latest telemetry/configuration inputs.
        """
        elapsed_ms = int((time.monotonic() - self.clock_synced_at) * 1000)
        current_ms = self.clock_base_ms + elapsed_ms
        self.data["timestamp"] = current_ms
        self.data["datetime"] = datetime.fromtimestamp(
            current_ms / 1000
        ).isoformat(timespec="seconds")

    def joystick_remaining(self):
        """
        Return remaining joystick lease seconds for active owner, or None when unlimited/not owned.
        """
        if self.joystick_owner is None or self.joystick_expires_at is None:
            return None
        remaining = int(round(self.joystick_expires_at - time.time()))
        return max(0, remaining)

    def adjust_joystick_owner_duration(self, delta_seconds):
        """
        Adjust bounded runtime parameters and keep values within safe limits.
        """
        if self.joystick_owner is None or self.joystick_expires_at is None:
            return False
        remaining = self.joystick_remaining()
        if remaining is None:
            return False
        next_remaining = clamp(int(remaining) + int(delta_seconds), 10, 3600)
        self.joystick_expires_at = time.time() + next_remaining
        return True

    def set_temporary_action(self, action, duration_seconds):
        """
        Apply a state mutation while preserving invariants relied upon by other runtime stages.
        """
        current_action = self.data["mission"]["action"]
        if current_action not in {"PHOTO", "VIDEO"}:
            self.last_action_before_temp = current_action
        self.data["mission"]["action"] = action
        self.action_expires_at = time.time() + max(0, duration_seconds)

    def clear_temporary_action_if_needed(self):
        """
        Cleanup ephemeral state to prevent stale references and cross-client side effects.
        """
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
        """
        Serialize internal state into stable transport payload structures for websocket clients.
        """
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
        """
        Build the canonical `SYS_INFO` payload consumed by remote/bridge UIs for role, queue, and ownership rendering.
        """
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
        """
        Persist normalized RX/TX packet telemetry for audit/debug visibility (memory ring + append-only log file).
        """
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
        """
        Return a bounded tail of exchange events for diagnostics endpoints without exposing unbounded history.
        """
        amount = clamp(int(limit), 1, EXCHANGE_LOG_LIMIT)
        return list(self.exchange_log)[-amount:]


# Single global state container used by HTTP handlers, WS handlers, and loops.
state = SystemState()


# -----------------------------------------------------------------------------
# Vision/autofocus coordinator (software-side state machine)
# -----------------------------------------------------------------------------
class VisionCoordinator:
    """
    Software autofocus/tracking coordinator.

    Responsibilities:
    - maintain autofocus pass state machine,
    - correlate frame focus scores with commanded/sensed focus positions,
    - expose tracking offsets and debug traces for UI and astronomy runtime.
    """
    TOTAL_FOCUS_STEPS = 10116.0

    def __init__(self, init_seconds=VISION_INIT_SECONDS):
        """
        Initialize object state used by cross-module orchestration and runtime safety gates.
        """
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
        self.pass_speeds = [1000.0, 600.0, 350.0]
        self.pass_windows = [self.TOTAL_FOCUS_STEPS, 3000.0, 1750.0]
        self.pass_ranges = []
        self.current_pass = 0
        self.pending_focus_command = None
        self.pending_sent_at = 0.0
        self.pending_ack1_timeout_s = 1.2
        self.pending_ack1_received = False
        self.pending_ack1_at = 0.0
        self.pending_action = None
        self.pending_target_step = 0.0
        self.pending_speed = 0.0
        self.pending_retry_count = 0
        self.pending_done_timeout_s = 20.0
        self.sweep_samples = []
        self.sweep_started_at = 0.0
        self.sweep_start_step = 0.0
        self.sweep_target_step = 0.0
        self.last_sweep_duration_s = 0.0
        self.last_sweep_measured_speed = 0.0
        self.last_sweep_sample_count = 0
        self.last_focus_error = ""
        self.autofocus_retry_count = 0
        self.last_best_sample_index = -1
        self.last_best_sample_score = -1.0
        self.focus_eval_counter = 0
        self.debug_events = deque(maxlen=VISION_DEBUG_EVENT_LIMIT)
        self.sweep_history = deque(maxlen=20)
        self.debug_log_file = VISION_DEBUG_LOG_FILE
        self._reset_pass_ranges()

    def _manual_to_step(self):
        """
        Convert manual focus percentage into absolute focus ticks.
        """
        return (float(self.manual_value) / 100.0) * self.TOTAL_FOCUS_STEPS

    def _record_focus_debug(self, event_type, **payload):
        """
        Append a structured autofocus debug event to memory and debug log file.
        """
        event = {
            "ts": utc_now_iso(),
            "type": str(event_type),
            "stage": str(self.stage),
            "active": bool(self.active),
            "ready": bool(self.ready),
            "pass_index": int(self.current_pass + 1),
            "pending": self.pending_focus_command,
        }
        event.update(payload or {})
        self.debug_events.append(event)
        try:
            self.debug_log_file.parent.mkdir(parents=True, exist_ok=True)
            with self.debug_log_file.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def _reset_pass_ranges(self):
        """
        Reset autofocus pass ranges to full-span defaults.
        """
        self.pass_ranges = [
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
            (0.0, self.TOTAL_FOCUS_STEPS),
        ]

    def _estimate_step_from_command_time(self, now_ts):
        """
        Estimate current focus step from command timing when direct position is not available.
        """
        start = float(self.sweep_start_step)
        target = float(self.sweep_target_step)
        distance = abs(target - start)
        speed = max(1.0, float(self.pass_speeds[self.current_pass]))
        expected_duration = max(0.001, distance / speed)
        ratio = _clamp_float((float(now_ts) - float(self.sweep_started_at)) / expected_duration, 0.0, 1.0)
        return float(start + ((target - start) * ratio))

    def _effective_capture_time(self, sample_ts):
        """
        Estimate center-of-exposure timestamp for each focus sample.
        """
        exposure_us = 0.0
        try:
            exposure_us = float(state.data.get("camera", {}).get("exposure", 0.0))
        except Exception:
            exposure_us = 0.0
        # Approximate center-of-exposure timestamp.
        return float(sample_ts) - max(0.0, (exposure_us * 1e-6) * 0.5), exposure_us

    def _clear_pending_focus(self):
        """
        Clear pending focus command tracking fields after completion/failure.
        """
        self.pending_focus_command = None
        self.pending_sent_at = 0.0
        self.pending_ack1_received = False
        self.pending_ack1_at = 0.0
        self.pending_action = None
        self.pending_target_step = 0.0
        self.pending_speed = 0.0
        self.pending_retry_count = 0

    def _resend_pending_focus(self, link, reason):
        """
        Retry the pending focus command and refresh watchdog timestamps.
        """
        action = int(self.pending_action or 0)
        target = float(self.pending_target_step)
        sent = False
        if action == 1:
            sent = bool(link.send_command(1, "FOC", int(round(target))))
        elif action == 2:
            sent = bool(link.send_command(2, "FOC", int(round(target)), float(self.pending_speed)))
        if sent:
            self.pending_retry_count += 1
            self.pending_sent_at = time.monotonic()
            self.pending_ack1_received = False
            self.pending_ack1_at = 0.0
            print(
                "[VISION] Re-sent focus command "
                f"(action={action}, target={int(round(target))}, reason={reason}, retry={self.pending_retry_count})."
            )
            self._record_focus_debug(
                "focus_command_resent",
                reason=str(reason),
                retry_count=int(self.pending_retry_count),
                action=action,
                target_step=round(target, 3),
                speed_ticks_per_s=round(float(self.pending_speed), 3),
            )
        else:
            self._record_focus_debug(
                "focus_command_resend_failed",
                reason=str(reason),
                retry_count=int(self.pending_retry_count),
                action=action,
                target_step=round(target, 3),
            )
            print(
                "[VISION] Failed to resend focus command "
                f"(action={action}, target={int(round(target))}, reason={reason})."
            )
        return sent

    def _update_tracking_error(self, frame):
        """
        Compute frame-center tracking offsets (dx/dy) from detected target contour.
        """
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
        """
        Update internal state using latest telemetry/configuration inputs.
        """
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
        """
        Update internal state using latest telemetry/configuration inputs.
        """
        self._update_tracking_error(frame)
        if frame is not None:
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self.focus_score = float(np.var(cv2.Laplacian(gray, cv2.CV_64F)))
            except Exception:
                self.focus_score = 0.0

        # Collect focus score samples only while the sweep command is active.
        if self.active and self.pending_focus_command == "SWEEP":
            sample_ts = time.monotonic()
            effective_ts, exposure_us = self._effective_capture_time(sample_ts)
            self.focus_eval_counter += 1
            sample = {
                "sample_id": int(self.focus_eval_counter),
                "ts": float(sample_ts),
                "effective_ts": float(effective_ts),
                "score": float(self.focus_score),
                "pass_index": int(self.current_pass + 1),
                "step_estimate_cmd": float(self._estimate_step_from_command_time(sample_ts)),
                "exposure_us": float(exposure_us),
            }
            self.sweep_samples.append(sample)
            self._record_focus_debug(
                "frame_evaluated",
                sample_id=sample["sample_id"],
                sample_index=int(len(self.sweep_samples) - 1),
                focus_score=round(sample["score"], 3),
                step_estimate_cmd=round(sample["step_estimate_cmd"], 3),
                exposure_us=round(float(exposure_us), 3),
            )

        if not self.active:
            self.stage = "waiting_hardware_calibration"
            if self.mode == "manual":
                self.current_focus_step = self._manual_to_step()

    def start_focus_calibration(self):
        """
        Reset autofocus pass state and transition into pass-1 preparation after hardware calibration is available.
        """
        self.active = True
        self.ready = False
        self.stage = "focus_prepare_pass_1"
        self.initialized_at = ""
        self.best_focus_score = -1.0
        self.best_focus_step = 0.0
        self.current_focus_step = 0.0
        self.current_pass = 0
        self.pending_focus_command = None
        self.pending_sent_at = 0.0
        self.pending_ack1_received = False
        self.pending_ack1_at = 0.0
        self.pending_action = None
        self.pending_target_step = 0.0
        self.pending_speed = 0.0
        self.pending_retry_count = 0
        self.pending_done_timeout_s = 20.0
        self.sweep_samples = []
        self.sweep_started_at = 0.0
        self.sweep_start_step = 0.0
        self.sweep_target_step = 0.0
        self.last_sweep_duration_s = 0.0
        self.last_sweep_measured_speed = 0.0
        self.last_sweep_sample_count = 0
        self.last_focus_error = ""
        self.autofocus_retry_count = 0
        self.last_best_sample_index = -1
        self.last_best_sample_score = -1.0
        self._reset_pass_ranges()
        self._record_focus_debug(
            "autofocus_start",
            pass_speeds=list(self.pass_speeds),
            pass_windows=list(self.pass_windows),
            pass_ranges=list(self.pass_ranges),
        )

    def stop_for_hardware_reset(self):
        """
        Stop autofocus workflow and clear pending focus command state after firmware reset/reboot.
        """
        # Hardware reboot invalidates current autofocus handshake state.
        self.active = False
        self.ready = False
        self.stage = "waiting_hardware_calibration"
        self.initialized_at = ""
        self.pending_focus_command = None
        self.pending_sent_at = 0.0
        self.pending_ack1_received = False
        self.pending_ack1_at = 0.0
        self.pending_action = None
        self.pending_target_step = 0.0
        self.pending_speed = 0.0
        self.pending_retry_count = 0
        self.pending_done_timeout_s = 20.0
        self.current_pass = 0
        self.sweep_samples = []
        self.sweep_started_at = 0.0
        self.last_focus_error = ""
        self._reset_pass_ranges()
        self._record_focus_debug("autofocus_stopped_hardware_reset")
        if self.mode == "manual":
            self.current_focus_step = self._manual_to_step()

    def _set_next_pass_range(self, pass_index, best_step):
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
        if pass_index not in (0, 1):
            return
        target_width = float(self._desired_pass_span(pass_index + 1))
        low, high = self._centered_window(float(best_step), target_width)
        self.pass_ranges[pass_index + 1] = (low, high)
        self._record_focus_debug(
            "pass_range_updated",
            next_pass=int(pass_index + 2),
            source_best_step=round(float(best_step), 3),
            target_width=round(target_width, 3),
            range_start=round(low, 3),
            range_end=round(high, 3),
        )

    def _desired_pass_span(self, pass_index):
        """
        Return configured span width for a given autofocus pass index.
        """
        if pass_index < 0 or pass_index >= len(self.pass_windows):
            return float(self.TOTAL_FOCUS_STEPS)
        return float(max(1.0, min(float(self.pass_windows[pass_index]), float(self.TOTAL_FOCUS_STEPS))))

    def _centered_window(self, center_step, width):
        """
        Build a centered [low, high] focus window clamped to physical focus limits.
        """
        target_width = float(max(1.0, min(float(width), float(self.TOTAL_FOCUS_STEPS))))
        pad = target_width / 2.0
        low = float(center_step) - pad
        high = float(center_step) + pad
        # Keep a constant sweep width by shifting the window when close to edges.
        if low < 0.0:
            high -= low
            low = 0.0
        if high > self.TOTAL_FOCUS_STEPS:
            low -= (high - self.TOTAL_FOCUS_STEPS)
            high = self.TOTAL_FOCUS_STEPS
        low = max(0.0, low)
        high = min(self.TOTAL_FOCUS_STEPS, high)
        # Final safety to guarantee exact requested width when mathematically possible.
        span = float(high - low)
        if abs(span - target_width) > 0.001 and target_width <= float(self.TOTAL_FOCUS_STEPS):
            if low <= 0.0:
                high = min(float(self.TOTAL_FOCUS_STEPS), low + target_width)
            elif high >= float(self.TOTAL_FOCUS_STEPS):
                low = max(0.0, high - target_width)
        return float(low), float(high)

    def _send_focus_position(self, link, target_step, stage_name):
        """
        Send a position-mode focus command and initialize pending command tracking.
        """
        sent = link.send_command(1, "FOC", int(round(target_step)))
        if sent:
            self.pending_focus_command = "REWIND" if "rewind" in stage_name else "LOCK"
            self.pending_sent_at = time.monotonic()
            self.pending_ack1_received = False
            self.pending_ack1_at = 0.0
            self.pending_action = 1
            self.pending_target_step = float(target_step)
            self.pending_speed = 0.0
            self.pending_retry_count = 0
            self.pending_done_timeout_s = 20.0
            self.stage = stage_name
            self.current_focus_step = float(target_step)
            self._record_focus_debug(
                "focus_position_command_sent",
                command_type=self.pending_focus_command,
                stage_name=str(stage_name),
                target_step=round(float(target_step), 3),
            )
        else:
            self._record_focus_debug(
                "focus_position_command_failed",
                stage_name=str(stage_name),
                target_step=round(float(target_step), 3),
            )
        return sent

    def _send_focus_sweep(self, link, start_step, target_step, speed):
        """
        Send a speed-mode focus sweep command and start sample collection state.
        """
        sent = link.send_command(2, "FOC", int(round(target_step)), float(speed))
        if sent:
            self.pending_focus_command = "SWEEP"
            self.pending_sent_at = time.monotonic()
            self.pending_ack1_received = False
            self.pending_ack1_at = 0.0
            self.pending_action = 2
            self.pending_target_step = float(target_step)
            self.pending_speed = float(speed)
            self.pending_retry_count = 0
            self.pending_done_timeout_s = 18.0
            self.stage = f"focus_sweep_pass_{self.current_pass + 1}"
            self.sweep_samples = []
            self.sweep_started_at = time.monotonic()
            self.sweep_start_step = float(start_step)
            self.sweep_target_step = float(target_step)
            self.current_focus_step = float(start_step)
            self._record_focus_debug(
                "focus_sweep_command_sent",
                pass_index=int(self.current_pass + 1),
                start_step=round(float(start_step), 3),
                target_step=round(float(target_step), 3),
                speed_ticks_per_s=round(float(speed), 3),
            )
        else:
            self._record_focus_debug(
                "focus_sweep_command_failed",
                pass_index=int(self.current_pass + 1),
                start_step=round(float(start_step), 3),
                target_step=round(float(target_step), 3),
                speed_ticks_per_s=round(float(speed), 3),
            )
        return sent

    def _finalize_sweep(self):
        """
        Map sweep samples to step estimates and pick the best-focus step for next action.
        """
        ended_at = time.monotonic()
        duration = max(0.001, ended_at - self.sweep_started_at)
        self.last_sweep_duration_s = duration
        distance = abs(self.sweep_target_step - self.sweep_start_step)
        self.last_sweep_measured_speed = distance / duration

        if not self.sweep_samples:
            self.last_focus_error = "No focus samples captured during sweep."
            self._record_focus_debug("focus_sweep_error", error=self.last_focus_error)
            return None

        direction = 1.0 if self.sweep_target_step >= self.sweep_start_step else -1.0
        best_step = self.sweep_start_step
        best_score = -1.0
        best_index = -1
        mapped_samples = []

        for idx, sample in enumerate(self.sweep_samples):
            sample_ts = float(sample.get("effective_ts", sample.get("ts", self.sweep_started_at)))
            score = float(sample.get("score", 0.0))
            ratio = _clamp_float((sample_ts - self.sweep_started_at) / duration, 0.0, 1.0)
            estimated_step = self.sweep_start_step + direction * (self.last_sweep_measured_speed * duration * ratio)
            if direction > 0:
                estimated_step = min(self.sweep_target_step, max(self.sweep_start_step, estimated_step))
            else:
                estimated_step = max(self.sweep_target_step, min(self.sweep_start_step, estimated_step))

            mapped = {
                "sample_id": int(sample.get("sample_id", idx)),
                "sample_index": int(idx),
                "relative_t_s": round(float(sample_ts - self.sweep_started_at), 6),
                "capture_t_s": round(float(sample.get("ts", sample_ts) - self.sweep_started_at), 6),
                "score": round(score, 6),
                "step_estimate_cmd": round(float(sample.get("step_estimate_cmd", estimated_step)), 6),
                "step_estimate_measured": round(float(estimated_step), 6),
                "exposure_us": round(float(sample.get("exposure_us", 0.0)), 3),
            }
            mapped_samples.append(mapped)
            self._record_focus_debug("frame_mapped", **mapped)

            if score >= best_score:
                best_score = score
                best_step = estimated_step
                best_index = idx

        self.last_sweep_sample_count = len(self.sweep_samples)
        self.best_focus_score = max(self.best_focus_score, best_score)
        self.best_focus_step = float(best_step)
        self.current_focus_step = float(best_step)
        self.last_best_sample_index = int(best_index)
        self.last_best_sample_score = float(best_score)
        history_entry = {
            "ts": utc_now_iso(),
            "pass_index": int(self.current_pass + 1),
            "start_step": round(float(self.sweep_start_step), 6),
            "target_step": round(float(self.sweep_target_step), 6),
            "speed_ticks_per_s": round(float(self.pass_speeds[self.current_pass]), 6),
            "duration_s": round(float(duration), 6),
            "sample_count": int(len(mapped_samples)),
            "best_sample_index": int(best_index),
            "best_score": round(float(best_score), 6),
            "best_step": round(float(best_step), 6),
            "samples": mapped_samples,
        }
        self.sweep_history.append(history_entry)
        self._record_focus_debug(
            "focus_sweep_finalized",
            pass_index=int(self.current_pass + 1),
            duration_s=round(float(duration), 6),
            sample_count=int(len(mapped_samples)),
            best_sample_index=int(best_index),
            best_score=round(float(best_score), 6),
            best_step=round(float(best_step), 6),
        )
        return best_step

    def handle_arduino_events(self, events, link):
        """
        Consume FOC-related ACK events (0/1/2), drive autofocus state transitions, and apply retry/error policy.
        """
        if not self.active:
            return

        for event in events:
            if event.get("kind") != "ack":
                continue
            if str(event.get("motor", "")).upper() != "FOC":
                continue

            code = event.get("code")
            action = event.get("action")
            raw_position = event.get("position")
            try:
                position = float(raw_position) if raw_position is not None else None
            except Exception:
                position = None
            self._record_focus_debug(
                "focus_ack_received",
                code=int(code) if code in (0, 1, 2) else code,
                action=action,
                motor=str(event.get("motor", "")),
                cmd=event.get("cmd"),
                position=position,
            )

            # Focus command rejected by firmware -> retry a limited number of times.
            if code == 0:
                failed_command = self.pending_focus_command
                self.autofocus_retry_count += 1
                self._clear_pending_focus()
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
                    self._record_focus_debug("focus_error", error=self.last_focus_error)
                continue

            if code == 1:
                if self.pending_focus_command and (
                    self.pending_action is None
                    or action in (self.pending_action, None)
                ):
                    if not self.pending_ack1_received:
                        self.pending_ack1_received = True
                        self.pending_ack1_at = time.monotonic()
                        pos_txt = (
                            str(int(round(position)))
                            if position is not None
                            else "unknown"
                        )
                        print(
                            f"[VISION] Ack 1 received for {self.pending_focus_command} "
                            f"(action={self.pending_action}, pos={pos_txt})."
                        )
                        self._record_focus_debug(
                            "focus_ack1_received",
                            pending=self.pending_focus_command,
                            action=self.pending_action,
                        )
                continue

            if code != 2:
                continue

            if self.pending_focus_command and (
                self.pending_action is None
                or action in (self.pending_action, None)
            ):
                if not self.pending_ack1_received:
                    self.pending_ack1_received = True
                    self.pending_ack1_at = time.monotonic()
                    self._record_focus_debug(
                        "focus_ack1_inferred_from_ack2",
                        pending=self.pending_focus_command,
                        action=self.pending_action,
                    )

            # Done for rewind/start positioning.
            if self.pending_focus_command == "REWIND":
                self.autofocus_retry_count = 0
                preferred_target_step = float(self.pass_ranges[self.current_pass][1])
                start_step = (
                    float(position)
                    if position is not None
                    else float(self.pass_ranges[self.current_pass][0])
                )
                desired_span = float(self._desired_pass_span(self.current_pass))
                target_step = preferred_target_step
                # Enforce full sweep width from the *actual* start position when possible.
                # If forward range does not fit, reverse sweep to keep full span.
                forward_target = float(start_step + desired_span)
                backward_target = float(start_step - desired_span)
                if 0.0 <= forward_target <= float(self.TOTAL_FOCUS_STEPS):
                    target_step = forward_target
                elif 0.0 <= backward_target <= float(self.TOTAL_FOCUS_STEPS):
                    target_step = backward_target
                else:
                    target_step = float(max(0.0, min(float(self.TOTAL_FOCUS_STEPS), preferred_target_step)))
                self.pass_ranges[self.current_pass] = (start_step, float(target_step))
                self._clear_pending_focus()
                self.current_focus_step = float(start_step)
                speed = self.pass_speeds[self.current_pass]
                effective_span = abs(float(target_step) - float(start_step))
                print(
                    "[VISION] Ack 2 received for REWIND "
                    f"(pass={self.current_pass + 1}, start={int(round(start_step))}, "
                    f"target={int(round(target_step))}, span={int(round(effective_span))}, "
                    f"speed={int(round(speed))})."
                )
                self._record_focus_debug(
                    "focus_rewind_done",
                    start_step=round(float(start_step), 6),
                    target_step=round(float(target_step), 6),
                    effective_span_ticks=round(float(effective_span), 6),
                    requested_span_ticks=round(float(desired_span), 6),
                    speed_ticks_per_s=round(float(speed), 6),
                )
                self._send_focus_sweep(link, start_step, target_step, speed)
                continue

            # Done for sweep move.
            if self.pending_focus_command == "SWEEP":
                self.autofocus_retry_count = 0
                if position is not None:
                    self.sweep_target_step = float(position)
                print(
                    "[VISION] Ack 2 received for SWEEP "
                    f"(pass={self.current_pass + 1}, final_pos={int(round(self.sweep_target_step))})."
                )
                self._clear_pending_focus()
                best_step = self._finalize_sweep()
                if best_step is None:
                    self.stage = "focus_error"
                    self.active = False
                    self._record_focus_debug("focus_error", error=self.last_focus_error)
                    continue

                if self.current_pass < 2:
                    self._set_next_pass_range(self.current_pass, best_step)
                    self.current_pass += 1
                    self.stage = f"focus_prepare_pass_{self.current_pass + 1}"
                else:
                    self._send_focus_position(link, best_step, "focus_lock_best")
                continue

            # Done for final lock.
            if self.pending_focus_command == "LOCK":
                self.autofocus_retry_count = 0
                self._clear_pending_focus()
                self.ready = True
                self.stage = "tracking"
                self.active = True
                self.initialized_at = utc_now_iso()
                print("[VISION] Ack 2 received for LOCK; autofocus complete.")
                if position is not None:
                    self.current_focus_step = float(position)
                    if self.mode == "auto":
                        self.best_focus_step = float(position)
                elif self.mode == "auto":
                    self.current_focus_step = self.best_focus_step
                else:
                    self.current_focus_step = self._manual_to_step()
                self._record_focus_debug(
                    "autofocus_complete",
                    best_step=round(float(self.best_focus_step), 6),
                    best_score=round(float(self.best_focus_score), 6),
                )

    def tick_autofocus(self, link):
        """
        Advance autofocus timing watchdogs and dispatch pending focus commands when state preconditions are met.
        """
        if not self.active or self.ready:
            return

        if self.pending_focus_command is not None:
            if self.pending_sent_at > 0.0:
                now = time.monotonic()
                sent_elapsed = now - float(self.pending_sent_at)
                if not self.pending_ack1_received:
                    if sent_elapsed > float(self.pending_ack1_timeout_s):
                        if self.pending_retry_count < 3 and self._resend_pending_focus(link, "ack1_timeout"):
                            return
                        self.last_focus_error = (
                            f"No ACK 1 for '{self.pending_focus_command}' after retries."
                        )
                        self.stage = "focus_error"
                        self.active = False
                        self._record_focus_debug("focus_error", error=self.last_focus_error)
                        return
                else:
                    done_elapsed = now - float(self.pending_ack1_at)
                    if done_elapsed > float(self.pending_done_timeout_s):
                        if self.pending_retry_count < 3 and self._resend_pending_focus(link, "ack2_timeout"):
                            return
                        self.last_focus_error = (
                            f"No ACK 2 for '{self.pending_focus_command}' within "
                            f"{self.pending_done_timeout_s:.1f}s."
                        )
                        self.stage = "focus_error"
                        self.active = False
                        self._record_focus_debug("focus_error", error=self.last_focus_error)
                        return
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
        """
        Expose the full vision/autofocus status snapshot mirrored directly into `state.data["vision_status"]`.
        """
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
            "pass_speeds": [round(float(v), 3) for v in self.pass_speeds],
            "pass_ranges": [
                [round(float(low), 3), round(float(high), 3)]
                for (low, high) in self.pass_ranges
            ],
            "pending_focus_command": self.pending_focus_command,
            "pending_action": self.pending_action,
            "pending_ack1_received": bool(self.pending_ack1_received),
            "pending_retry_count": int(self.pending_retry_count),
            "pending_done_timeout_s": round(float(self.pending_done_timeout_s), 3),
            "pending_elapsed_s": (
                round(float(max(0.0, time.monotonic() - self.pending_sent_at)), 3)
                if self.pending_focus_command and self.pending_sent_at > 0.0
                else 0.0
            ),
            "best_sample_index": int(self.last_best_sample_index),
            "best_sample_score": round(float(self.last_best_sample_score), 3),
            "debug_events_count": int(len(self.debug_events)),
            "history_count": int(len(self.sweep_history)),
        }

    def debug_snapshot(self, event_limit=500, history_limit=10, sample_limit=2000):
        """
        Return deep autofocus debug material (events/history/current samples) for `/debug/vision` inspection.
        """
        e_lim = max(1, min(int(event_limit), 50000))
        h_lim = max(1, min(int(history_limit), 50))
        s_lim = max(1, min(int(sample_limit), 100000))
        history_out = []
        for item in list(self.sweep_history)[-h_lim:]:
            samples = list(item.get("samples", []))
            row = {k: v for k, v in item.items() if k != "samples"}
            row["samples_total"] = int(len(samples))
            row["samples"] = samples[-s_lim:]
            history_out.append(row)
        return {
            "state": self.to_payload(),
            "recent_events": list(self.debug_events)[-e_lim:],
            "history": history_out,
            "current_sweep_samples": list(self.sweep_samples)[-s_lim:],
            "debug_log_file": str(self.debug_log_file),
        }


def build_capture_paths(target_id, capture_kind):
    """
    Generate deterministic mission-scoped photo/video filesystem paths (date folder + sanitized target folder).
    """
    now = datetime.now()
    day_folder = now.strftime("%Y-%m-%d")
    mission_folder = sanitize_fs_name(target_id, fallback="STANDBY")
    stamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    prefix = "Photo" if capture_kind == "photo" else "Video"
    extension = ".png" if capture_kind == "photo" else ".avi"

    base_dir = CAPTURE_ROOT / day_folder / mission_folder
    filename = f"{prefix}_{mission_folder}_{stamp}{extension}"
    return base_dir, base_dir / filename


# -----------------------------------------------------------------------------
# Runtime orchestration state (hardware calibration + autofocus gating)
# -----------------------------------------------------------------------------
class RuntimeOrchestrator:
    """
    Cross-subsystem startup orchestration state holder.

    Tracks hardware calibration handshake progress, autofocus readiness,
    and retry/error-recovery timing between loop iterations.
    """
    def __init__(self):
        """
        Initialize object state used by cross-module orchestration and runtime safety gates.
        """
        self.hw_calibration_requested = False
        self.hw_calibration_requested_at = 0.0
        self.hw_last_progress_at = 0.0
        self.hw_calibration_accepted = False
        self.hw_calibration_started = False
        self.hw_calibrated = False
        self.hw_calibration_retries = 0
        self.hw_next_retry_at = 0.0
        self.focus_only_mode = False
        self.focus_calibration_started = False
        self.focus_calibrated = False
        self.last_handshake_code = None
        self.error_recovery_last_at = 0.0

    def reset(self):
        """
        Reset orchestrator flags before restarting startup/calibration pipeline.
        """
        self.__init__()


# Global runtime singletons (initialized during app startup).
astro_runtime = None
vision_runtime = VisionCoordinator()
runtime_orchestrator = RuntimeOrchestrator()
catalog_refresh_deadline = 0.0
catalog_last_hash = ""


# -----------------------------------------------------------------------------
# Camera capture/publish service
# -----------------------------------------------------------------------------
class CameraManager:
    """
    Camera acquisition and media writer service.

    Handles:
    - live frame acquisition (hardware or simulation),
    - stream-safe frame conversions,
    - photo/video capture workers and status reporting.
    """
    def __init__(self, mode="hardware"):
        """
        Initialize object state used by cross-module orchestration and runtime safety gates.
        """
        self.mode = mode
        self.frame = None
        self.raw_frame = None
        self.save_frame = None
        self.running = True
        self.camera = None
        self.video = None
        self.current_image_type = None
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
        """
        Start the camera capture thread.
        """
        self.thread.start()

    def get_frame(self):
        """
        Return latest encoded MJPEG frame bytes for stream endpoint.
        """
        with self.lock:
            return self.frame

    def get_raw_frame_copy(self):
        """
        Return a copy of the latest runtime BGR frame for processing.
        """
        with self.lock:
            if self.raw_frame is None:
                return None
            return self.raw_frame.copy()

    def get_save_frame_copy(self):
        """
        Return a copy of the highest-fidelity frame used for photo/video capture.
        """
        with self.lock:
            if self.save_frame is None:
                return None
            return self.save_frame.copy()    

    def apply_camera_settings(self, settings):
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
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
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
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
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
        if asi is None:
            return False
        try:
            asi.init(ASI_LIB)
            if asi.get_num_cameras() <= 0:
                return False
            self.camera = asi.Camera(0)
            desired_type, _ = self._desired_hardware_image_type()
            self.camera.set_roi(
                width=VIDEO_WIDTH,
                height=VIDEO_HEIGHT,
                bins=1,
                image_type=desired_type,
            )
            self.current_image_type = int(desired_type)
            self.apply_camera_settings(state.data["camera"])
            self.camera.start_video_capture()
            return True
        except Exception as exc:
            print(f"Hardware camera init failed: {exc}")
            self.camera = None
            return False

    def _desired_hardware_image_type(self):
        """
        Choose hardware pixel format based on camera mode and current mission target.
        """
        if asi is None:
            return None, "no_asi"
        camera_type = str(state.data.get("camera", {}).get("type", "auto")).lower()
        if camera_type == "planet":
            return int(asi.ASI_IMG_RGB24), "planet"
        if camera_type in {"deep_sky", "deep"}:
            return int(asi.ASI_IMG_RAW16), "deep_sky"
        mission_target = state.data.get("mission", {}).get("target", "STANDBY")
        if mission_target_is_solar(mission_target):
            return int(asi.ASI_IMG_RGB24), "auto_solar"
        return int(asi.ASI_IMG_RAW16), "auto_deep"

    def _set_hardware_image_type_if_needed(self):
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
        if self.camera is None or asi is None:
            return
        desired_type, reason = self._desired_hardware_image_type()
        if desired_type is None:
            return
        if self.current_image_type == int(desired_type):
            return
        with self.settings_lock:
            try:
                try:
                    self.camera.stop_video_capture()
                except Exception:
                    pass
                self.camera.set_roi(
                    width=VIDEO_WIDTH,
                    height=VIDEO_HEIGHT,
                    bins=1,
                    image_type=int(desired_type),
                )
                self.camera.start_video_capture()
                self.current_image_type = int(desired_type)
                print(
                    "[CAM] Image type switched "
                    f"(mode={state.data.get('camera', {}).get('type', 'auto')}, reason={reason}, "
                    f"target={state.data.get('mission', {}).get('target', 'STANDBY')}, "
                    f"type={self.current_image_type})."
                )
            except Exception as exc:
                print(f"[CAM] Failed to switch image type: {exc}")

    def _prepare_frames_for_runtime(self, frame):
        """
        Returns:
        - web_frame: uint8 BGR frame for UI stream + vision processing
        - save_frame: highest-fidelity frame for captures
        """
        if frame is None:
            return None, None

        if frame.ndim == 2:
            # Bayer stream (typically RAW16 in deep mode): debayer for web,
            # keep 16-bit debayerized data for high-quality captures.
            if frame.dtype == np.uint16:
                try:
                    debayer_16 = cv2.cvtColor(frame, cv2.COLOR_BAYER_RG2BGR)
                except Exception:
                    debayer_16 = np.stack([frame, frame, frame], axis=-1)
                web_frame = cv2.convertScaleAbs(debayer_16, alpha=(255.0 / 65535.0))
                return web_frame, debayer_16
            frame_u8 = frame.astype(np.uint8, copy=False)
            try:
                debayer_8 = cv2.cvtColor(frame_u8, cv2.COLOR_BAYER_RG2BGR)
            except Exception:
                debayer_8 = np.stack([frame_u8, frame_u8, frame_u8], axis=-1)
            return debayer_8, debayer_8

        if frame.dtype == np.uint16:
            web_frame = cv2.convertScaleAbs(frame, alpha=(255.0 / 65535.0))
            return web_frame, frame

        return frame, frame

    @staticmethod
    def _prepare_frame_for_writer(frame):
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
        if frame is None:
            return None
        out = frame
        if out.ndim == 2:
            out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        if out.dtype == np.uint16:
            out = cv2.convertScaleAbs(out, alpha=(255.0 / 65535.0))
        elif out.dtype != np.uint8:
            out = np.clip(out, 0, 255).astype(np.uint8)
        if out.shape[1] != VIDEO_WIDTH or out.shape[0] != VIDEO_HEIGHT:
            out = cv2.resize(out, (VIDEO_WIDTH, VIDEO_HEIGHT))
        return out

    def _init_simulation(self):
        """
        Internal helper used by camera/runtime pipelines to keep data formats and modes consistent.
        """
        if os.path.exists("simulation.mp4"):
            self.video = cv2.VideoCapture("simulation.mp4")

    def _capture_loop(self):
        """
        Main capture loop: acquire frames, publish stream bytes, and update shared frame buffers.
        """
        print(f"[{self.mode.upper()}] Camera loop starting")

        if self.mode == "hardware" and not self._init_hardware_camera():
            self.mode = "sim"

        if self.mode == "sim":
            self._init_simulation()

        while self.running:
            frame = None

            if self.mode == "hardware" and self.camera is not None:
                try:
                    self._set_hardware_image_type_if_needed()
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

            web_frame, save_frame = self._prepare_frames_for_runtime(frame)
            if web_frame is None:
                time.sleep(0.04)
                continue
            
            success, jpeg = cv2.imencode(
                ".jpg", web_frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if success:
                with self.lock:
                    self.frame = jpeg.tobytes()
                    self.raw_frame = web_frame.copy()
                    self.save_frame = save_frame.copy() if save_frame is not None else web_frame.copy()

            try:
                h, w = web_frame.shape[:2]
                payload = [
                    b"video_feed",
                    str(time.time()).encode("utf-8"),
                    f"{h},{w},3".encode("utf-8"),
                    web_frame.tobytes(),
                ]
                self.zmq_socket.send_multipart(payload, flags=zmq.NOBLOCK)
            except Exception:
                pass

            time.sleep(0.04)

    def capture_photo(self, output_path):
        """
        Save a still frame to disk and return (ok, error_message).
        """
        frame = self.get_save_frame_copy()
        if frame is None:
            return False, "No video frame available yet."

        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        suffix = output_path.suffix.lower()
        params = []
        if suffix == ".png":
            params = [cv2.IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION]
        elif suffix in {".jpg", ".jpeg"}:
            params = [cv2.IMWRITE_JPEG_QUALITY, 100]        
        ok = cv2.imwrite(str(output_path), frame, params)
        if not ok:
            return False, "Failed to save photo."
        return True, ""

    def _record_video_worker(self, output_path, duration_seconds, fps):
        """
        Background worker that writes frames to video file for requested duration.
        """
        try:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            writer = None
            selected_codec = None
            for codec in ("FFV1", "HFYU", "MJPG", "mp4v"):            
                fourcc = cv2.VideoWriter_fourcc(*codec)
                candidate = cv2.VideoWriter(
                    str(output_path),
                    fourcc,
                    float(fps),
                    (VIDEO_WIDTH, VIDEO_HEIGHT),
                )
                if candidate.isOpened():
                    writer = candidate
                    selected_codec = codec
                    break
                candidate.release()
            if writer is None:
                raise RuntimeError("Video writer could not be opened.")
            print(
                f"[CAPTURE] Video recording started (codec={selected_codec}, path={output_path})."
            )
            started = time.monotonic()
            interval = 1.0 / max(1.0, float(fps))
            while self.running and (time.monotonic() - started) < float(duration_seconds):
                frame = self.get_save_frame_copy()
                if frame is not None:
                    frame_for_writer = self._prepare_frame_for_writer(frame)
                    if frame_for_writer is not None:
                        writer.write(frame_for_writer)
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
        """
        Start asynchronous video recording if no recording is currently active.
        """
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
        """
        Return current recording state and last recording diagnostics.
        """
        with self.recording_lock:
            return {
                "active": bool(self.recording_active),
                "last_error": self.last_recording_error,
                "last_path": self.last_recording_path,
            }


cam = None


# -----------------------------------------------------------------------------
# WebSocket exchange helpers
# -----------------------------------------------------------------------------
def record_exchange(direction, payload, ws=None, delivered=True):
    """
    Persist normalized RX/TX packet telemetry for audit/debug visibility (memory ring + append-only log file).
    """
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
    """
    Send websocket payload safely while logging delivery success/failure for observability.
    """
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
    """
    Send initial bootstrap payload so a newly connected client can render immediately.
    """
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
    """
    Fan out `SYS_INFO` updates to all active clients after role/queue/ownership changes.
    """
    for ws in list(state.clients.keys()):
        await safe_send(ws, state.serialize_system_info(ws))


async def broadcast_catalog():
    """
    Fan out latest visible mission catalog to all clients when catalog hash changes.
    """
    payload = {"type": "CATALOG_UPDATE", "catalog": state.catalog}
    for ws in list(state.clients.keys()):
        await safe_send(ws, payload)


async def broadcast_telemetry():
    """
    Fan out current telemetry snapshot at loop-defined cadence or after command-side mutations.
    """
    payload = {"type": "TELEMETRY", "data": state.data}
    for ws in list(state.clients.keys()):
        await safe_send(ws, payload)


async def trigger_emergency_stop():
    """
    Trigger critical system behavior and propagate safety updates to all clients.
    """
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
    """
    Forward queued approval request payloads to commander, with graceful fallback if unavailable.
    """
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
    """
    Grant joystick ownership (with optional lease) and revoke previous owner if required.
    """
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
    """
    Revoke current joystick owner, reset vector state, and optionally advance queue.
    """
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
    """
    Advance joystick queue and notify next eligible client when ownership becomes available.
    """
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
    """
    Apply admin queue operations (move/remove/duration edits) to joystick queue entries.
    """
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
    """
    Adjust bounded runtime parameters and keep values within safe limits.
    """
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
    """
    Return guard conditions used to gate command execution and privilege-sensitive operations.
    """
    return packet_type in {
        "GPS_UPDATE",
        "SYNC_CLOCK",
        "TIMESTAMP",
        "DATETIME",
        "JOYSTICK",
        "MOTOR_SETTINGS",
        "MISSION",
        "COMMAND",
        "CAMERA_SETTINGS",
        "FOCUS_SETTINGS",
        "PHOTO_REQUEST",
        "VIDEO_REQUEST",
        "RUNTIME_MODES",
        "AUTOFOCUS_RECALIBRATE",
        "AUTOFOCUS_TEST",
        "SERIAL_RAW",
        "EMERGENCY_STOP",
    }


def refresh_system_status():
    """
    Fuse astronomy, vision, and Arduino status into a single UI-facing `system` readiness contract.
    """
    # Pull live subsystem status snapshots (with robust fallback when astronomy runtime is missing).
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

    # Compute high-level readiness gates consumed by UI lock/unlock logic.
    hardware_calibrated = bool(runtime_orchestrator.hw_calibrated)
    focus_ready = bool(vision_status.get("ready", False))
    astro_ready = bool(astro_status.get("ready", False))

    # Do not block general UI controls on autofocus completion.
    controls_ready = bool(hardware_calibrated and astro_ready)
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

    # Mirror detailed subsystem statuses into telemetry payload.
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
    # Catalog is considered ready once altitude-bearing entries are available
    # or at least one target is present.
    catalog_has_altitude = any(
        isinstance(entry.get("altitude_deg"), (int, float))
        for entries in state.catalog.values()
        for entry in entries
    )

    state.data["system"] = {
        "hardware_calibrated": hardware_calibrated,
        "vision_ready": bool(vision_status["ready"]),
        "astronomic_ready": astro_ready,
        "controls_ready": controls_ready,
        "catalog_ready": bool(
            catalog_has_altitude
            or any(len(items) for items in state.catalog.values())
        ),
        "message": message,
    }
    return controls_ready


def compute_catalog_hash(catalog):
    """
    Build stable hash text for catalog change detection.
    """
    try:
        return json.dumps(catalog, sort_keys=True, ensure_ascii=True)
    except Exception:
        return str(catalog)


async def refresh_catalog_if_due(force=False):
    """
    Refresh astronomy visible catalog using adaptive cadence and broadcast when content changed.
    """
    global catalog_refresh_deadline, catalog_last_hash
    now = time.monotonic()
    if not force and astro_runtime is not None:
        try:
            astro_status = astro_runtime.status()
            cache_rows = int((astro_status.get("catalog_cache") or {}).get("rows", 0))
            if bool(astro_status.get("ready")) and bool(astro_status.get("skyfield_ready")) and cache_rows <= 0:
                force = True
        except Exception:
            pass
    if not force and now < catalog_refresh_deadline:
        return

    if astro_runtime is None:
        empty_catalog = {tab: [] for tab in CATALOG}
        if state.catalog != empty_catalog:
            state.catalog = empty_catalog
            catalog_last_hash = compute_catalog_hash(state.catalog)
            await broadcast_catalog()
        catalog_refresh_deadline = now + 30.0
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
    has_altitude = any(
        isinstance(entry.get("altitude_deg"), (int, float))
        for entries in next_catalog.values()
        for entry in entries
    )
    if not has_altitude:
        catalog_refresh_deadline = now + 2.0
    else:
        catalog_refresh_deadline = now + max(5.0, float(delay or 60.0))


async def handle_photo_capture_request(source_ws=None):
    """
    Execute photo capture workflow and persist capture metadata into shared telemetry.
    """
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
    """
    Execute video capture workflow and persist recording metadata/status into shared telemetry.
    """
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
    """
    Central command router: validates init gates, mutates shared state, and triggers motor/capture side effects.
    """
    global catalog_refresh_deadline
    # Packet type is the canonical switch key used by every frontend command path.
    ptype = pkt.get("type")
    # When True, telemetry is rebroadcast at function end to reflect state mutations.
    should_broadcast = False

    # During startup/init phases, only an allowlist of commands may pass.
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
        # Real-time joystick vector updates (already privilege-gated upstream).
        data = pkt.get("data", {})
        state.data["joystick"] = {
            "x": float(data.get("x", 0.0)),
            "y": float(data.get("y", 0.0)),
        }
        should_broadcast = True

    elif ptype == "EMERGENCY_STOP":
        # Hard safety stop: transitions mission action and closes active sessions.
        state.data["mission"]["action"] = "EMERGENCY_STOP"
        state.action_expires_at = None
        await trigger_emergency_stop()
        return

    elif ptype == "MISSION":
        # Mission updates are validated against currently visible catalog targets.
        requested_target = pkt.get("target", "STANDBY")
        if requested_target != "STANDBY":
            visible_ids = {
                str(entry.get("id"))
                for entries in state.catalog.values()
                for entry in entries
                if entry.get("id") is not None
            }
            if str(requested_target) not in visible_ids:
                print(
                    "[MISSION] Rejected target not visible in catalog "
                    f"(target={requested_target}, visible_count={len(visible_ids)})."
                )
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
        # High-level action command wrapper (photo/video/manual action switch).
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
        # GPS sync updates observer coordinates and invalidates catalog refresh deadline.
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
        # Time sync updates both local runtime clock and astronomy runtime epoch.
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
        # Camera settings are bounded and pushed into live capture backend.
        camera = state.data["camera"]
        payload = pkt.get("data", {})
        if "exposure" in payload:
            camera["exposure"] = clamp(int(payload["exposure"]), 1, 10000000)
        if "gain" in payload:
            camera["gain"] = clamp(int(payload["gain"]), 0, 600)
        requested_type = payload.get("type")
        if requested_type in {"auto", "deep_sky", "deep", "planet"}:
            camera["type"] = "deep_sky" if requested_type == "deep" else requested_type
        cam.apply_camera_settings(camera)
        should_broadcast = True

    elif ptype == "MOTOR_SETTINGS":
        # Tracking feature toggles (camera offsets + earth rotation compensation).
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
        # Focus controls support both percent and absolute tick-domain updates.
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

    elif ptype in {"AUTOFOCUS_RECALIBRATE", "AUTOFOCUS_TEST"}:
        # Software autofocus kickoff path (normal mode or focus-only diagnostic mode).
        force_focus_only = bool(
            ptype == "AUTOFOCUS_TEST"
            or pkt.get("force_focus_only")
            or pkt.get("focus_only")
        )
        if not force_focus_only and not runtime_orchestrator.hw_calibrated:
            if source_ws is not None:
                await safe_send(
                    source_ws,
                    {"type": "TOAST", "msg": "Hardware calibration is not complete yet."},
                )
            return
        if force_focus_only:
            runtime_orchestrator.focus_only_mode = True
            runtime_orchestrator.hw_calibration_requested = True
            runtime_orchestrator.hw_calibration_accepted = True
            runtime_orchestrator.hw_calibrated = True
            if source_ws is not None:
                await safe_send(
                    source_ws,
                    {"type": "TOAST", "msg": "Autofocus test mode enabled (RA/DEC calibration bypassed)."},
                )
        else:
            runtime_orchestrator.focus_only_mode = False
        vision_runtime.start_focus_calibration()
        runtime_orchestrator.focus_calibration_started = True
        runtime_orchestrator.focus_calibrated = False
        should_broadcast = True

    elif ptype == "VIDEO_REQUEST":
        # Direct video request shortcut from UI.
        duration = clamp(int(pkt.get("duration", 10)), 1, 3600)
        should_broadcast = await handle_video_capture_request(
            duration=duration, source_ws=source_ws
        )

    elif ptype == "PHOTO_REQUEST":
        # Direct photo request shortcut from UI.
        should_broadcast = await handle_photo_capture_request(source_ws=source_ws)

    elif ptype == "RUNTIME_MODES":
        # Runtime mode toggles (simulation flags, camera mode notice).
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

    elif ptype == "SERIAL_RAW":
        # Manual/raw serial passthrough for debugging firmware interactions.
        command = str(pkt.get("command", "")).strip()
        sent = False
        if astro_runtime is not None and command:
            try:
                sent = bool(astro_runtime.motor_link.send_interactive(command))
            except Exception:
                sent = False
        if source_ws is not None:
            await safe_send(
                source_ws,
                {
                    "type": "TOAST",
                    "msg": (
                        f"Serial command sent: {command}"
                        if sent
                        else "Failed to send serial command."
                    ),
                },
            )
        should_broadcast = False

    # Keep client telemetry consistent after mutating any shared runtime state.
    if should_broadcast:
        await broadcast_telemetry()


async def handle_request_response(ws, pkt):
    """
    Apply commander/bridge decision to a pending request and execute corresponding approval outcome.
    """
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
    """
    Primary websocket session handler for auth, approval workflow, command routing, and disconnect cleanup.
    """
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
            # Only textual websocket frames are part of this JSON protocol.
            if msg.type != WSMsgType.TEXT:
                continue

            try:
                pkt = json.loads(msg.data)
            except json.JSONDecodeError:
                # Ignore malformed frames instead of dropping the client.
                continue

            record_exchange("rx", pkt, ws=ws, delivered=True)
            ptype = pkt.get("type")

            if ptype == "LOGIN_BRIDGE":
                # Bridge login upgrades this session to admin-like bridge role.
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
                # Lightweight profile/name updates for UI presence.
                state.set_client_name(ws, pkt.get("name", "Anon"))
                await broadcast_system_info()
                continue

            if ptype == "REQUEST_COMMANDER":
                # Request commander ownership directly or via queue workflow.
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
                # Bridge-only forced commander reassignment.
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
                # Observer action requiring commander approval.
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
                # Joystick privilege request path (direct grant or queued approval).
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
                # Commander/bridge decision for a pending request.
                await handle_request_response(ws, pkt)
                continue

            if ptype == "TOGGLE_MUTE":
                # Commander/bridge controls request popup mute behavior.
                if state.can_admin(ws):
                    state.mute_notifications = bool(pkt.get("mute", False))
                    await broadcast_system_info()
                continue

            if ptype == "JOYSTICK_QUEUE_MANAGE":
                # Queue reorder/remove actions from bridge/commander tooling.
                await manage_joystick_queue(ws, pkt)
                continue

            if ptype == "JOYSTICK_OWNER_TIME":
                # Lease extension/reduction actions for active joystick owner.
                await adjust_joystick_owner_duration(ws, pkt)
                continue

            if ptype == "REVOKE_JOYSTICK":
                # Admin revoke command.
                if state.can_admin(ws):
                    await revoke_joystick(reason="revoked_by_admin", advance_queue=True)
                continue

            if ptype == "RELEASE_JOYSTICK":
                # Owner/admin voluntary release command.
                if ws == state.joystick_owner or state.can_admin(ws):
                    await revoke_joystick(reason="released", advance_queue=True)
                continue

            if ptype == "JOYSTICK":
                # High-frequency joystick vectors: strictly gated by ownership/commander.
                if ws == state.joystick_owner or state.is_commander(ws):
                    await execute_command(pkt, source_ws=ws)
                continue

            # Fallback: all other command types require commander/bridge privileges.
            if state.can_admin(ws) or state.is_commander(ws):
                await execute_command(pkt, source_ws=ws)

    finally:
        # Ensure ownership/queues are repaired even on abrupt disconnect.
        lost_owner = ws == state.joystick_owner
        state.unregister(ws)
        if lost_owner:
            await grant_next_joystick()
        await broadcast_system_info()
        await broadcast_telemetry()

    return ws


async def video_feed(request):
    """
    Serve MJPEG stream endpoint consumed by WebUI live-preview widgets.
    """
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
    """
    Serve emergency or remote page depending on global safety-stop state.
    """
    if state.emergency_stop:
        return web.FileResponse("./public/emergency.html")
    return web.FileResponse("./public/remote.html")


async def page_remote(request):
    """
    Serve remote control page when not in emergency-stop mode.
    """
    if state.emergency_stop:
        raise web.HTTPFound("/")
    return web.FileResponse("./public/remote.html")


async def page_bridge(request):
    """
    Serve bridge/admin page when not in emergency-stop mode.
    """
    if state.emergency_stop:
        raise web.HTTPFound("/")
    return web.FileResponse("./public/bridge.html")


async def debug_state(request):
    """
    Expose consolidated runtime debug snapshot over HTTP for diagnostics.
    """
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
    """
    Expose bounded packet exchange log for UI/network debugging.
    """
    try:
        limit = int(request.query.get("limit", "200"))
    except ValueError:
        limit = 200
    return web.json_response({"events": state.exchange_snapshot(limit=limit)})


async def debug_serial(request):
    """
    Expose bounded serial RX/TX and motor-link status diagnostics.
    """
    try:
        limit = int(request.query.get("limit", "200"))
    except ValueError:
        limit = 200
    if astro_runtime is None:
        return web.json_response(
            {"events": [], "status": {"available": False, "message": "Astronomic runtime unavailable."}}
        )
    try:
        link = astro_runtime.motor_link
        return web.json_response(
            {
                "events": link.serial_snapshot(limit=limit),
                "status": {
                    "available": True,
                    "motor_link": link.status(),
                },
            }
        )
    except Exception as exc:
        return web.json_response(
            {"events": [], "status": {"available": False, "message": str(exc)}},
            status=500,
        )


async def debug_serial_send(request):
    """
    Forward raw interactive serial commands via debug endpoint and report send status.
    """
    if astro_runtime is None:
        return web.json_response({"ok": False, "error": "Astronomic runtime unavailable."}, status=503)
    try:
        payload = await request.json()
    except Exception:
        payload = {}
    command = str(payload.get("command", "")).strip()
    if not command:
        return web.json_response({"ok": False, "error": "Missing 'command'."}, status=400)
    try:
        sent = bool(astro_runtime.motor_link.send_interactive(command))
        return web.json_response({"ok": sent, "command": command, "motor_link": astro_runtime.motor_link.status()})
    except Exception as exc:
        return web.json_response({"ok": False, "error": str(exc)}, status=500)


async def debug_vision(request):
    """
    Expose detailed autofocus debug state/history/event traces for analysis.
    """
    try:
        event_limit = int(request.query.get("event_limit", "500"))
    except ValueError:
        event_limit = 500
    try:
        history_limit = int(request.query.get("history_limit", "10"))
    except ValueError:
        history_limit = 10
    try:
        sample_limit = int(request.query.get("sample_limit", "2000"))
    except ValueError:
        sample_limit = 2000
    try:
        snapshot = vision_runtime.debug_snapshot(
            event_limit=event_limit,
            history_limit=history_limit,
            sample_limit=sample_limit,
        )
        return web.json_response(snapshot)
    except Exception as exc:
        return web.json_response({"error": str(exc)}, status=500)


async def handle_runtime_orchestration(vision_payload):
    """
    Coordinate startup calibration handshake, firmware ACK processing, and autofocus orchestration against motor link events.
    """
    # Main hardware/vision orchestration checkpoint.
    # This function is intentionally centralized so startup retries, firmware
    # acknowledgements, and autofocus sequencing stay in one place.
    if astro_runtime is None:
        return
    if runtime_orchestrator.focus_only_mode:
        runtime_orchestrator.hw_calibration_requested = True
        runtime_orchestrator.hw_calibration_accepted = True
        runtime_orchestrator.hw_calibrated = True

    link = astro_runtime.motor_link
    # Pull a bounded chunk of serial/motor-link events each loop tick.
    events = link.drain_events(limit=200)
    boot_seen = False
    calibration_progress_seen = False
    calibration_ready_seen = False

    # First pass over debug lines: infer coarse firmware state transitions.
    for event in events:
        if event.get("kind") != "debug":
            continue
        line_lower = str(event.get("line", "")).lower()
        if not line_lower:
            continue
        if "dbg,boot" in line_lower:
            boot_seen = True
        if "mount=calibrating" in line_lower or "dbg,calib" in line_lower:
            calibration_progress_seen = True
        if "mount=ready" in line_lower or "calibration_done" in line_lower:
            calibration_ready_seen = True

    # Firmware reboot/boot banner means all calibration/autofocus handshakes must restart.
    if boot_seen:
        had_runtime_state = bool(
            runtime_orchestrator.hw_calibration_requested
            or runtime_orchestrator.hw_calibrated
            or runtime_orchestrator.focus_calibration_started
        )
        runtime_orchestrator.reset()
        vision_runtime.stop_for_hardware_reset()
        if had_runtime_state:
            print("[HW CAL] Arduino boot/reset detected; restarting initialization pipeline.")

    # Second pass over ACK events: update explicit calibration handshake state machine.
    for event in events:
        if event.get("kind") != "ack":
            continue
        code = event.get("code")
        action = event.get("action")
        motor = str(event.get("motor", "")).upper()
        if code not in (0, 1, 2):
            continue
        is_hw_event = bool(
            action == 3
            or (
                action is None
                and motor not in {"RA", "DEC", "FOC"}
                and runtime_orchestrator.hw_calibration_requested
                and not runtime_orchestrator.hw_calibrated
            )
        )
        if not is_hw_event:
            continue

        event_now = time.monotonic()
        runtime_orchestrator.last_handshake_code = code
        if code == 1:
            runtime_orchestrator.hw_calibration_accepted = True
            runtime_orchestrator.hw_calibration_started = True
            runtime_orchestrator.hw_calibration_requested_at = event_now
            runtime_orchestrator.hw_last_progress_at = event_now
            runtime_orchestrator.hw_next_retry_at = event_now + CALIBRATION_COMMAND_TIMEOUT_S
            print("[HW CAL] Ack 1 received; calibration started/confirmed.")
        elif code == 2:
            was_calibrated = bool(runtime_orchestrator.hw_calibrated)
            runtime_orchestrator.hw_calibration_accepted = True
            runtime_orchestrator.hw_calibration_started = True
            runtime_orchestrator.hw_calibrated = True
            runtime_orchestrator.hw_last_progress_at = event_now
            runtime_orchestrator.error_recovery_last_at = 0.0
            if not was_calibrated:
                print("[HW CAL] Ack 2 received; calibration complete.")
        elif code == 0 and not runtime_orchestrator.hw_calibrated:
            runtime_orchestrator.hw_calibration_accepted = False
            runtime_orchestrator.hw_next_retry_at = event_now + 1.0

    # If firmware reports ERROR after calibration, force controlled handshake reset.
    now = time.monotonic()
    link_status = link.status()
    link_system_state = str(link_status.get("system_state", "")).upper()
    if link_system_state == "ERROR" and runtime_orchestrator.hw_calibrated:
        if (now - float(runtime_orchestrator.error_recovery_last_at)) >= 5.0:
            runtime_orchestrator.error_recovery_last_at = now
            runtime_orchestrator.hw_calibrated = False
            runtime_orchestrator.hw_calibration_requested = False
            runtime_orchestrator.hw_calibration_accepted = False
            runtime_orchestrator.hw_calibration_started = False
            runtime_orchestrator.hw_next_retry_at = now
            print("[HW CAL] Mount entered ERROR state; restarting calibration handshake.")

    # Hardware calibration handshake with graceful retry.
    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and calibration_progress_seen
    ):
        runtime_orchestrator.hw_calibration_accepted = True
        runtime_orchestrator.hw_calibration_started = True
        runtime_orchestrator.hw_last_progress_at = now
        runtime_orchestrator.hw_next_retry_at = now + CALIBRATION_COMMAND_TIMEOUT_S
    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and calibration_ready_seen
        and runtime_orchestrator.hw_calibration_started
    ):
        runtime_orchestrator.hw_calibration_accepted = True
        runtime_orchestrator.hw_calibrated = True
        runtime_orchestrator.hw_last_progress_at = now
        print("[HW CAL] Calibration completed from debug state (mount=READY).")

    should_send_calib = False
    if not runtime_orchestrator.hw_calibration_requested:
        should_send_calib = True
    elif (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and not runtime_orchestrator.hw_calibration_accepted
        and now >= runtime_orchestrator.hw_next_retry_at
        and runtime_orchestrator.hw_calibration_retries > 0
    ):
        should_send_calib = True

    if should_send_calib and not runtime_orchestrator.hw_calibrated:
        sent = link.send_command(3)
        runtime_orchestrator.hw_calibration_requested = bool(sent)
        runtime_orchestrator.hw_calibration_accepted = False
        runtime_orchestrator.hw_calibration_started = False
        runtime_orchestrator.hw_calibration_requested_at = now
        runtime_orchestrator.hw_last_progress_at = now
        if sent:
            runtime_orchestrator.hw_calibration_retries += 1
            runtime_orchestrator.hw_next_retry_at = now + CALIBRATION_COMMAND_TIMEOUT_S
            print(
                f"Startup calibration request sent (3), try #{runtime_orchestrator.hw_calibration_retries}."
            )
        else:
            runtime_orchestrator.hw_next_retry_at = now + 1.0
            print("Failed to send startup calibration command (3) to Arduino.")

    # Watchdog: accepted calibration with no progress within timeout -> retry path.
    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and runtime_orchestrator.hw_calibration_accepted
    ):
        last_activity = max(
            float(runtime_orchestrator.hw_calibration_requested_at),
            float(runtime_orchestrator.hw_last_progress_at),
        )
        elapsed = time.monotonic() - last_activity
        if elapsed > CALIBRATION_COMMAND_TIMEOUT_S:
            # Do not force ERROR here; keep system alive and let UI remain accessible.
            print(
                "Warning: hardware calibration stalled "
                f"({elapsed:.1f}s since last progress), retrying command (3)."
            )
            runtime_orchestrator.hw_calibration_accepted = False
            runtime_orchestrator.hw_next_retry_at = time.monotonic() + 1.0

    # Backward-compatible fallback when firmware prints state but does not emit "2".
    last_dbg = link.status().get("last_debug_line", "")
    dbg_lower = str(last_dbg).lower()
    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and runtime_orchestrator.hw_calibration_started
        and ("calibrat" in dbg_lower)
    ):
        runtime_orchestrator.hw_calibration_accepted = True
        runtime_orchestrator.hw_last_progress_at = time.monotonic()
    if (
        runtime_orchestrator.hw_calibration_requested
        and not runtime_orchestrator.hw_calibrated
        and runtime_orchestrator.hw_calibration_started
        and ("mount=ready" in dbg_lower or "calibration_done" in dbg_lower)
    ):
        runtime_orchestrator.hw_calibrated = True
        runtime_orchestrator.hw_calibration_accepted = True
        runtime_orchestrator.hw_last_progress_at = time.monotonic()
        print("[HW CAL] Calibration completed from fallback debug line.")

    # Run autofocus handshake against Arduino command acks.
    if runtime_orchestrator.focus_calibration_started:
        vision_runtime.handle_arduino_events(events, link)
        vision_runtime.tick_autofocus(link)

    if runtime_orchestrator.focus_calibration_started and vision_runtime.to_payload().get("ready", False):
        if not runtime_orchestrator.focus_calibrated:
            print("[VISION] Autofocus calibration complete.")
        runtime_orchestrator.focus_calibrated = True

    # Apply final command gates used by astronomy runtime command emission.
    tracking_enabled = bool(
        runtime_orchestrator.hw_calibrated
        and astro_runtime.ready
    )
    autofocus_busy = bool(
        runtime_orchestrator.focus_calibration_started
        and not runtime_orchestrator.focus_calibrated
    )
    focus_gate_open = bool(runtime_orchestrator.hw_calibrated and not autofocus_busy)
    astro_runtime.set_command_gates(
        hardware_ready=runtime_orchestrator.hw_calibrated,
        # Focus commands are allowed once hardware is calibrated, but we pause
        # operator-driven focus sync while autofocus sweeps are running to avoid
        # command contention on FOC.
        focus_ready=focus_gate_open,
        tracking_enabled=tracking_enabled,
    )


async def background_state_loop(app):
    """
    Periodic runtime loop that advances clocks, vision/astro ticks, orchestration, catalog refresh, and broadcast cadence.
    """
    # High-frequency runtime loop:
    # - updates camera/vision snapshots
    # - runs orchestration and astronomy tick
    # - refreshes catalogs/system flags
    # - pushes telemetry/system packets to UI clients
    last_telemetry_push = 0.0
    last_system_info_push = 0.0
    while True:
        # Keep software clock moving even without explicit client sync packets.
        state.update_clock()
        # Expire temporary action labels (PHOTO/VIDEO) once their display window ends.
        state.clear_temporary_action_if_needed()

        # Pull latest camera frame for vision scoring and tracking-offset extraction.
        frame = cam.get_raw_frame_copy()
        vision_runtime.update_from_frame(frame)
        # Orchestration can change autofocus state (commands/acks), so we refresh
        # payload after running it.
        await handle_runtime_orchestration(vision_runtime.to_payload())
        vision_payload = vision_runtime.to_payload()
        state.data["vision_status"] = vision_payload

        if astro_runtime is not None:
            try:
                # Feed camera-derived offsets + focus state into astronomy planner.
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
                # Tick astronomy operator to generate motor commands from target+offsets.
                astro_runtime.tick(state.data)
            except Exception as exc:
                print(f"Astronomic tick failed: {exc}")

        # Mirror recorder worker state into telemetry for UI status chips.
        recording_status = cam.recording_status()
        state.data["capture"]["recording"] = bool(recording_status["active"])
        if recording_status["last_error"]:
            state.data["capture"]["last_error"] = recording_status["last_error"]
        if recording_status["last_path"]:
            state.data["capture"]["last_video_path"] = recording_status["last_path"]

        refresh_system_status()

        # Auto-revoke joystick lease when countdown reaches zero.
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

        # Fixed-cycle pacing for this coordinator loop.
        await asyncio.sleep(BACKGROUND_LOOP_INTERVAL_S)


async def start_background_tasks(app):
    """
    Startup lifecycle hook: initialize runtimes, synchronize initial state, and launch background orchestration loop.
    """
    # Startup bootstrap:
    # 1) initialize logs/runtime modes
    # 2) create astronomy runtime + initial sync
    # 3) initialize focus settings/catalog state
    # 4) launch background state loop
    global astro_runtime, catalog_refresh_deadline, catalog_last_hash
    runtime_orchestrator.reset()
    # Ensure all runtime log/storage targets exist before subsystems start writing.
    CAPTURE_ROOT.mkdir(parents=True, exist_ok=True)
    EXCHANGE_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    EXCHANGE_LOG_FILE.touch(exist_ok=True)
    SERIAL_EXCHANGE_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    SERIAL_EXCHANGE_LOG_FILE.touch(exist_ok=True)
    VISION_DEBUG_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    VISION_DEBUG_LOG_FILE.touch(exist_ok=True)

    env_sim_arduino = os.environ.get("STARTRACK_SIM_ARDUINO", "").strip().lower()
    # `STARTRACK_SIM_ARDUINO` overrides mode-based default when explicitly set.
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
    # Support explicit serial port overrides; otherwise auto-detect likely Arduino port.
    if requested_port.lower() == "auto":
        arduino_port = detect_arduino_port("COM6" if os.name == "nt" else "/dev/ttyUSB0")
    else:
        arduino_port = requested_port
    arduino_baud = int(os.environ.get("STARTRACK_ARDUINO_BAUD", "115200"))
    state.data["runtime_modes"]["arduino_port"] = arduino_port

    if AstronomicOperator is not None:
        try:
            # Instantiate astronomy runtime with shared catalog + serial configuration.
            astro_runtime = AstronomicOperator(
                base_catalog=CATALOG,
                latitude=state.data["gps"]["lat"],
                longitude=state.data["gps"]["lon"],
                altitude_m=state.data["gps"]["alt"],
                data_root=str(PROJECT_ROOT / "Software" / "Raspberry Pi" / "Astronomic Operator"),
                simulate_arduino=simulate_arduino,
                arduino_port=arduino_port,
                arduino_baudrate=arduino_baud,
                serial_log_file=str(SERIAL_EXCHANGE_LOG_FILE),
                serial_log_max=SERIAL_EXCHANGE_LOG_LIMIT,
            )
        except Exception as exc:
            print(f"Astronomic runtime startup failed: {exc}")
            astro_runtime = None
    else:
        astro_runtime = None

    if astro_runtime is not None:
        try:
            # Immediately seed astronomy runtime with current GPS/time values.
            astro_runtime.set_location(
                state.data["gps"]["lat"],
                state.data["gps"]["lon"],
                state.data["gps"].get("alt", 0.0),
            )
            astro_runtime.set_time(
                timestamp_ms=state.data.get("timestamp"),
                datetime_value=state.data.get("datetime"),
            )
        except Exception as exc:
            print(f"Astronomic runtime initial sync failed: {exc}")

    focus_cfg = state.data.get("focus", {})
    # Normalize legacy/manual focus fields to consistent tick+percent representation.
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
        # Start with gates closed until hardware+focus orchestration opens them.
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
    """
    Shutdown lifecycle hook: cancel async loop and close astronomy/serial resources gracefully.
    """
    # Graceful shutdown for async background loop and serial runtime.
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
    app.router.add_get("/debug/serial", debug_serial)
    app.router.add_post("/debug/serial/send", debug_serial_send)
    app.router.add_get("/debug/vision", debug_vision)
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
