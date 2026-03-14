import argparse
import asyncio
import json
import os
import ssl
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from aiohttp import WSMsgType, web
import zmq
try:
    import zwoasi as asi
except ImportError:
    asi = None


SSL_CERT = "cert.pem"
SSL_KEY = "key.pem"
ASI_LIB = "/usr/lib/libASICamera2.so"
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
JPEG_QUALITY = 60
BRIDGE_PIN = "1234"

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


def parse_datetime_to_ms(value):
    if not value:
        return None
    try:
        normalized = value.replace("Z", "+00:00")
        return int(datetime.fromisoformat(normalized).timestamp() * 1000)
    except (TypeError, ValueError):
        return None


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
            "focus": {"mode": "auto", "manual_value": 50},
            "capture": {
                "video_duration": 10,
                "last_photo_at": "",
                "last_video_request_at": "",
                "last_video_duration": 10,
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

    def sync_clock(self, timestamp_ms=None, datetime_value=None):
        derived_ms = parse_datetime_to_ms(datetime_value)
        effective_ms = timestamp_ms or derived_ms or int(time.time() * 1000)
        self.clock_base_ms = int(effective_ms)
        self.clock_synced_at = time.monotonic()
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


state = SystemState()


class CameraManager:
    def __init__(self, mode="hardware"):
        self.mode = mode
        self.frame = None
        self.running = True
        self.camera = None
        self.video = None
        self.lock = threading.Lock()
        self.settings_lock = threading.Lock()
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        context = zmq.Context()
        self.zmq_socket = context.socket(zmq.PUB)
        self.zmq_socket.bind("ipc:///tmp/video_feed")

    def start(self):
        self.thread.start()

    def get_frame(self):
        with self.lock:
            return self.frame

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
                    self.zmq_socket.send(frame.tobytes())
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

            time.sleep(0.04)


cam = None


async def safe_send(ws, payload):
    if ws is None or ws.closed:
        return False
    try:
        await ws.send_json(payload)
        return True
    except Exception as exc:
        print(f"WebSocket send failed: {exc}")
        return False


async def send_full_state(ws):
    await safe_send(
        ws,
        {
            "type": "FULL_STATE",
            "data": state.data,
            "catalog": CATALOG,
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


async def execute_command(pkt, source_ws=None):
    ptype = pkt.get("type")
    should_broadcast = False

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
        state.data["mission"]["target"] = pkt.get("target", "STANDBY")
        state.data["mission"]["action"] = pkt.get("action", "GOTO")
        state.action_expires_at = None
        if state.data["mission"]["action"] not in {"PHOTO", "VIDEO"}:
            state.last_action_before_temp = state.data["mission"]["action"]
        should_broadcast = True

    elif ptype == "COMMAND":
        action = pkt.get("action", "--")
        if action == "PHOTO":
            state.data["capture"]["last_photo_at"] = utc_now_iso()
            state.set_temporary_action("PHOTO", 3)
        elif action == "VIDEO":
            duration = clamp(int(pkt.get("duration", 10)), 1, 3600)
            state.data["capture"]["video_duration"] = duration
            state.data["capture"]["last_video_duration"] = duration
            state.data["capture"]["last_video_request_at"] = utc_now_iso()
            state.set_temporary_action("VIDEO", duration)
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
        state.sync_clock(timestamp_ms=timestamp_ms, datetime_value=datetime_value)
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
        if "manual_value" in payload:
            focus["manual_value"] = clamp(int(payload["manual_value"]), 0, 100)
        should_broadcast = True

    elif ptype == "VIDEO_REQUEST":
        duration = clamp(int(pkt.get("duration", 10)), 1, 3600)
        state.data["capture"]["video_duration"] = duration
        state.data["capture"]["last_video_duration"] = duration
        state.data["capture"]["last_video_request_at"] = utc_now_iso()
        state.set_temporary_action("VIDEO", duration)
        should_broadcast = True

    elif ptype == "PHOTO_REQUEST":
        state.data["capture"]["last_photo_at"] = utc_now_iso()
        state.set_temporary_action("PHOTO", 3)
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


async def background_state_loop(app):
    while True:
        state.update_clock()
        state.clear_temporary_action_if_needed()
        if (
            state.joystick_owner is not None
            and state.joystick_expires_at is not None
            and time.time() >= state.joystick_expires_at
        ):
            await revoke_joystick(reason="expired", advance_queue=True)
        await broadcast_telemetry()
        await broadcast_system_info()
        await asyncio.sleep(1)


async def start_background_tasks(app):
    app["state_loop"] = asyncio.create_task(background_state_loop(app))


async def cleanup_background_tasks(app):
    app["state_loop"].cancel()
    try:
        await app["state_loop"]
    except asyncio.CancelledError:
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="hardware")
    args = parser.parse_args()

    cam = CameraManager(args.mode)
    cam.start()

    app = web.Application()
    app.router.add_get("/", page_root)
    app.router.add_get("/remote", page_remote)
    app.router.add_get("/bridge", page_bridge)
    app.router.add_get("/stream", video_feed)
    app.router.add_get("/ws", handle_websocket)
    app.router.add_static("/static", "./public/static")
    app.on_startup.append(start_background_tasks)
    app.on_cleanup.append(cleanup_background_tasks)

    ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ssl_ctx.load_cert_chain(SSL_CERT, SSL_KEY)

    print("--- STARTRACK READY ---")
    web.run_app(app, port=443, ssl_context=ssl_ctx)
