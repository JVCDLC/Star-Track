import cv2
import numpy as np
import zmq
import time
import threading
import queue
import serial
from pathlib import Path
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

# ==========================================
# SYSTEM CONSTANTS
# ==========================================
TOTAL_MOTOR_STEPS = 10116  # Total physical steps of the sweep range
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
UI_HEIGHT = 150
SIM_VIDEO_DEFAULT = Path(__file__).resolve().parents[1] / "WebUI" / "simulation.mp4"

def draw_hud(data, current_step, state, focus_score, current_frame):
    """Draws a dedicated UI panel at the bottom of the screen with a clean graph."""
    ui = np.zeros((UI_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
    
    # Grid lines and UI background
    cv2.rectangle(ui, (0, 0), (VIDEO_WIDTH, UI_HEIGHT), (20, 20, 20), -1)
    cv2.line(ui, (0, 0), (VIDEO_WIDTH, 0), (255, 255, 255), 2)
    
    # Text Information (Left side)
    cv2.putText(ui, f"State: {state}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(ui, f"Motor Step: {current_step:.1f} / {TOTAL_MOTOR_STEPS:.0f}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(ui, f"Video Frame: {current_frame}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 100), 2)
    cv2.putText(ui, f"Focus Score: {focus_score:.1f}", (10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    # Graph Drawing (Right side)
    g_x, g_y = 300, 20
    g_w, g_h = VIDEO_WIDTH - g_x - 20, UI_HEIGHT - 40
    cv2.rectangle(ui, (g_x, g_y), (g_x + g_w, g_y + g_h), (40, 40, 40), -1)
    
    if len(data) >= 2:
        steps = [d[0] for d in data]
        scores = [d[1] for d in data]
        min_s, max_s = min(steps), max(steps)
        max_score = max(scores) if max(scores) > 0 else 1
        
        # Plot curve
        for i in range(1, len(data)):
            x1 = int(g_x + ((steps[i-1] - min_s) / (max_s - min_s + 1e-5)) * g_w)
            y1 = int(g_y + g_h - (scores[i-1] / max_score) * g_h)
            x2 = int(g_x + ((steps[i] - min_s) / (max_s - min_s + 1e-5)) * g_w)
            y2 = int(g_y + g_h - (scores[i] / max_score) * g_h)
            cv2.line(ui, (x1, y1), (x2, y2), (0, 255, 255), 2)
            
        # Draw current position marker (Red Line)
        if min_s <= current_step <= max_s:
            cx = int(g_x + ((current_step - min_s) / (max_s - min_s + 1e-5)) * g_w)
            cv2.line(ui, (cx, g_y), (cx, g_y + g_h), (0, 0, 255), 2)
            
    cv2.putText(ui, "Focus Graph", (g_x, g_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
    return ui

def put_text_outline(img, text, pos, color=(0, 255, 0)):
    """Draws highly visible text over the video feed."""
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3) # Outline
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)     # Inner

# ==========================================
# 1. VISION PROCESSING
# ==========================================
class VisionProcessor:
    def __init__(self, scale=0.5):
        self.scale = scale

    def process_frame(self, frame):
        # Force frame to standard size for consistency
        frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
        original = frame.copy()
        
        small = cv2.resize(frame, None, fx=self.scale, fy=self.scale)
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)

        _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None

        if contours:
            largest = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest)
            center = (int(x / self.scale), int(y / self.scale))
            cv2.circle(original, center, 5, (0, 0, 255), -1)

        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        if np.any(mask > 0):
            focus_score = np.var(laplacian[mask > 0])
        else:
            focus_score = 0.0

        return original, focus_score, center

# ==========================================
# 2. AUTOFOCUS ANALYZER
# ==========================================
class AutofocusAnalyzer:
    def __init__(self, threshold_ratio=0.5, smoothing_window=5):
        self.data =[]  
        self.threshold_ratio = threshold_ratio
        self.smoothing_window = smoothing_window

    def add_data_point(self, step, focus_score):
        self.data.append((step, focus_score))

    def analyze_sweep(self, pass_number):
        if len(self.data) < self.smoothing_window:
            return None, None

        self.data.sort(key=lambda x: x[0]) 
        steps = np.array([x[0] for x in self.data])
        scores = np.array([x[1] for x in self.data])

        kernel = np.ones(self.smoothing_window) / self.smoothing_window
        smoothed_scores = np.convolve(scores, kernel, mode='valid')
        
        trim = self.smoothing_window // 2
        valid_steps = steps[trim : -trim]
        if len(valid_steps) == 0: valid_steps = steps 

        max_idx = np.argmax(smoothed_scores)
        best_step = valid_steps[max_idx]
        
        if pass_number == 1:
            left_side = smoothed_scores[:max_idx]
            right_side = smoothed_scores[max_idx:]
            start_step = valid_steps[np.argmin(left_side)] if len(left_side) > 0 else valid_steps[0]
            end_step = valid_steps[max_idx + np.argmin(right_side)] if len(right_side) > 0 else valid_steps[-1]
            
        elif pass_number == 2:
            max_score = smoothed_scores[max_idx]
            threshold = max_score * self.threshold_ratio
            left_side = smoothed_scores[:max_idx]
            start_step = valid_steps[np.where(left_side < threshold)[0][-1]] if np.any(left_side < threshold) else valid_steps[0]
            right_side = smoothed_scores[max_idx:]
            end_step = valid_steps[max_idx + np.where(right_side < threshold)[0][0]] if np.any(right_side < threshold) else valid_steps[-1]
            
        else:
            start_step, end_step = best_step, best_step

        return float(best_step), (float(min(start_step, end_step)), float(max(start_step, end_step)))

    def reset(self):
        self.data =[]

    def analyze_timed_sweep(self, pass_number, start_step, target_step, start_time, end_time):
        """
        Convert (timestamp, focus_score) samples to estimated motor steps using the measured
        sweep duration, then reuse the standard pass analysis.
        """
        duration = max(1e-3, float(end_time) - float(start_time))
        distance = float(target_step) - float(start_step)

        converted = []
        for ts, score in self.data:
            ratio = (float(ts) - float(start_time)) / duration
            ratio = max(0.0, min(1.0, ratio))
            est_step = float(start_step) + (distance * ratio)
            converted.append((est_step, score))

        self.data = converted
        return self.analyze_sweep(pass_number), duration

# ==========================================
# 3. HARDWARE CONTROLLER (DUAL PID PHYSICS)
# ==========================================
class PID:
    def __init__(self, kp, ki, kd, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(-self.out_max, min(self.out_max, output))

class ArduinoController:
    def __init__(self, simulate=False, port='auto', baudrate=115200):
        self.simulate = simulate
        self.cmd_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.running = True

        # --- Simulated Physics State ---
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.target_pos = 0.0
        self.target_speed = 0.0
        self.start_pos = 0.0
        self.state = "IDLE"  
        
        # Dual PID Control
        # Position PID (Outputs Target Speed)
        self.pos_pid = PID(kp=4.0, ki=0.0, kd=0.1, out_max=800.0)
        # Speed PID (Outputs Acceleration)
        self.vel_pid = PID(kp=3.0, ki=0.5, kd=0.05, out_max=2000.0) 

        if not self.simulate:
            if port in (None, "", "auto"):
                port = self._detect_port()
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1)
            except Exception as e:
                print(f"Error opening serial port: {e}. Falling back to simulation.")
                self.simulate = True

        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _detect_port(self):
        if list_ports is None:
            return '/dev/ttyUSB0'
        ports = list(list_ports.comports())
        if not ports:
            return '/dev/ttyUSB0'
        def score(p):
            text = " ".join([str(p.device), str(p.description), str(p.hwid)]).lower()
            s = 0
            if "arduino" in text: s += 20
            if "ch340" in text or "wch" in text: s += 15
            if "usb serial" in text or "ttyusb" in text or "ttyacm" in text: s += 10
            if str(p.device).upper().startswith("COM"): s += 5
            return s
        return sorted(ports, key=score, reverse=True)[0].device

    def get_current_position(self):
        return self.current_pos

    def send_command(self, action, motor, value="", value2=""):
        if value2 != "":
            cmd_str = f"{action},{motor},{value},{value2}"
        else:
            cmd_str = f"{action},{motor},{value}"
        self.cmd_queue.put(cmd_str)
        print(f"[Arduino TX] {cmd_str}")

    def check_response(self):
        try:
            return self.response_queue.get_nowait()
        except queue.Empty:
            return None

    def _worker(self):
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = max(0.001, current_time - last_time)
            last_time = current_time

            if not self.cmd_queue.empty() and self.state == "IDLE":
                cmd = self.cmd_queue.get()
                
                if self.simulate:
                    parts = cmd.split(',')
                    action = int(parts[0])
                    
                    if action == 1:
                        # Cmd 1: Move to Position (Uses Position -> Speed PID)
                        self.target_pos = float(parts[2])
                        self.pos_pid.integral = 0
                        self.vel_pid.integral = 0
                        self.state = "PID_MOVE"
                        self.response_queue.put("1")  # accepted
                        
                    elif action == 2:
                        # Cmd 2 (refactored protocol): 2,FOC,<target_ticks>,<ticks_per_second>
                        self.target_pos = float(parts[2])
                        self.target_speed = float(parts[3])
                        self.start_pos = self.current_pos
                        self.vel_pid.integral = 0
                        self.state = "SWEEP_MOVE"
                        self.response_queue.put("1")  # accepted
                else:
                    self.ser.write((cmd + '\n').encode('utf-8'))

            # --- PHYSICS ENGINE UPDATE ---
            if self.simulate:
                if self.state == "PID_MOVE":
                    # Cascade: Position Error -> Target Velocity
                    pos_err = self.target_pos - self.current_pos
                    target_vel = self.pos_pid.compute(pos_err, dt)
                    
                    # Cascade: Velocity Error -> Acceleration
                    vel_err = target_vel - self.current_vel
                    accel = self.vel_pid.compute(vel_err, dt)
                    
                    self.current_vel += accel * dt
                    self.current_pos += self.current_vel * dt
                    
                    # Arrival logic (Close enough & not moving fast)
                    if abs(pos_err) < 1.0 and abs(self.current_vel) < 5.0:
                        self.current_pos = self.target_pos
                        self.current_vel = 0.0
                        self.state = "IDLE"
                        self.response_queue.put("2")
                        print("[Arduino RX - SIM] 2")

                elif self.state == "SWEEP_MOVE":
                    direction = 1 if self.target_pos > self.start_pos else -1
                    target_vel = self.target_speed * direction
                    
                    # Speed PID to Acceleration
                    vel_err = target_vel - self.current_vel
                    accel = self.vel_pid.compute(vel_err, dt)
                    
                    self.current_vel += accel * dt
                    self.current_pos += self.current_vel * dt
                    
                    # Immediate stop upon crossing Endpoint
                    if (direction == 1 and self.current_pos >= self.target_pos) or \
                       (direction == -1 and self.current_pos <= self.target_pos):
                        self.current_pos = self.target_pos
                        self.current_vel = 0.0
                        self.state = "IDLE"
                        self.response_queue.put("2")
                        print("[Arduino RX - SIM] 2")
                        
            if not self.simulate and self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                if response in ["0", "1", "2"]:
                    self.response_queue.put(response)
                    print(f"[Arduino RX] {response}")
                    
            time.sleep(0.005) # Run physics loop at 200Hz

# ==========================================
# 4. FRAME/STEP CORRELATOR
# ==========================================
class CameraSource:
    def __init__(self, use_zmq=False, source="ipc:///tmp/video_feed"):
        self.use_zmq = use_zmq
        
        if self.use_zmq:
            self.context = zmq.Context()
            self.sub_socket = self.context.socket(zmq.SUB)
            self.sub_socket.connect(source)
            self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"video_feed")
            self.sub_socket.setsockopt(zmq.CONFLATE, 1)
            self.total_frames = None
        else:
            self.cap = cv2.VideoCapture(source)
            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def get_frame(self, current_step):
        if self.use_zmq:
            try:
                topic, ts_bytes, shape_bytes, frame_bytes = self.sub_socket.recv_multipart(flags=zmq.NOBLOCK)
                h, w, c = map(int, shape_bytes.decode('utf-8').split(','))
                frame = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((h, w, c))
                return 0, frame # Realtime
            except zmq.Again:
                return 0, None
        else:
            # MAP MOTOR STEP -> EXACT FRAME OF VIDEO
            step_ratio = current_step / TOTAL_MOTOR_STEPS
            frame_idx = int(step_ratio * (self.total_frames - 1))
            frame_idx = max(0, min(frame_idx, self.total_frames - 1))
            
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
            ret, frame = self.cap.read()
            if not ret:
                return frame_idx, np.zeros((480, 640, 3), dtype=np.uint8) 
            return frame_idx, frame

# ==========================================
# 5. MAIN SYSTEM LOOP
# ==========================================
def main(simulate_video, simulate_arduino, video_source):
    processor = VisionProcessor(scale=0.5)
    analyzer = AutofocusAnalyzer()
    arduino = ArduinoController(simulate=simulate_arduino)
    camera = CameraSource(use_zmq=not simulate_video, source=video_source)

    state = "INIT_PASS" 
    
    # PASS STRUCTURE: (Start_Pos, Target_Pos, Target_Speed)
    sweep_passes =[
        (0.0, TOTAL_MOTOR_STEPS, 1000.0), # Sweep 1: Scan entire range
        (0.0, 0.0, 600.0),               # Sweep 2: Dynamic
        (0.0, 0.0, 350.0)                 # Sweep 3: Dynamic
    ] 
    
    current_pass_idx = 0
    next_sweep_range = (0, 0)
    best_focus_step = 0.0
    sweep_start_time = 0.0
    sweep_start_step = 0.0
    sweep_target_step = 0.0

    print("\nStarting Vision System...")

    while True:
        # 1. Fetch physical step
        current_step = arduino.get_current_position()

        # 2. Map physical step to frame
        current_frame, frame = camera.get_frame(current_step)
        if frame is None:
            time.sleep(0.01)
            continue

        # 3. Vision Processing
        display_frame, focus_score, center = processor.process_frame(frame)
        
        # --- STATE MACHINE ---
        if state == "INIT_PASS":
            if current_pass_idx == 0:
                pass_start, pass_target, pass_speed = sweep_passes[0]
            else:
                pass_start, pass_target = next_sweep_range
                pass_speed = sweep_passes[current_pass_idx][2]
                
            print(f"--- Preparing Focus Pass {current_pass_idx + 1} ---")
            while arduino.check_response() is not None: pass
            
            # CMD 1: Go to Start point using Pos PID
            arduino.send_command(1, "FOC", int(round(pass_start)))
            state = "WAIT_REWIND"
            
        elif state == "WAIT_REWIND":
            if arduino.check_response() == "2":
                state = "START_SWEEP"

        elif state == "START_SWEEP":
            if current_pass_idx == 0:
                pass_start, pass_target, pass_speed = sweep_passes[0]
            else:
                pass_start, pass_target = next_sweep_range
                pass_speed = sweep_passes[current_pass_idx][2]

            print(f"--- Pass {current_pass_idx + 1} ---")
            while arduino.check_response() is not None: pass
            
            # CMD 2: Sweep using Speed PID (Speed, Endpoint)
#            arduino.send_command(2, "FOC", int(round(pass_target)), float(pass_speed))
            
            analyzer.reset()
            sweep_start_time = time.monotonic()
            sweep_start_step = float(pass_start)
            sweep_target_step = float(pass_target)
            state = "SWEEPING"

        elif state == "SWEEPING":
            # Store focus by timestamp; map to step at end using measured sweep time.
            analyzer.add_data_point(time.monotonic(), focus_score)
            
            if arduino.check_response() == "2":
                sweep_end_time = time.monotonic()
                (result_best, result_range), measured_duration = analyzer.analyze_timed_sweep(
                    current_pass_idx + 1,
                    sweep_start_step,
                    sweep_target_step,
                    sweep_start_time,
                    sweep_end_time,
                )
                measured_speed = abs(sweep_target_step - sweep_start_step) / max(measured_duration, 1e-3)
                print(
                    f"Sweep telemetry: duration={measured_duration:.3f}s, "
                    f"measured_speed={measured_speed:.2f} ticks/s, "
                    f"samples={len(analyzer.data)}"
                )
                
                if result_best is not None:
                    print(f"Pass {current_pass_idx + 1} complete. Best step: {result_best:.2f}\n")
                    current_pass_idx += 1
                    
                    if current_pass_idx < len(sweep_passes):
                        # Dynamic tight bounding based on pass results
                        padding = 150.0 if current_pass_idx == 1 else 50.0
                        new_start = max(0.0, min(result_range[0], result_best - padding))
                        new_target = min(TOTAL_MOTOR_STEPS, max(result_range[1], result_best + padding))
                        
                        next_sweep_range = (new_start, new_target)
                        state = "INIT_PASS"
                    else:
                        best_focus_step = result_best
                        print(f"=== AUTOFOCUS COMPLETE. Locking to step {int(best_focus_step)} ===")
                        state = "LOCK_FOCUS"
                else:
                    print("Sweep failed (not enough data). Retrying current pass...\n")
                    state = "INIT_PASS" 

        elif state == "LOCK_FOCUS":
            while arduino.check_response() is not None: pass
            
            # CMD 1: Final approach to absolute best spot using Position PID
            arduino.send_command(1, "FOC", int(round(best_focus_step)))
            state = "WAIT_LOCK"

        elif state == "WAIT_LOCK":
            if arduino.check_response() == "2":
                state = "TRACKING"

        # --- TRACKING / CENTERING LOGIC ---
        elif state == "TRACKING":
            if center:
                h, w = display_frame.shape[:2]
                dx, dy = (w // 2) - center[0], (h // 2) - center[1]
                put_text_outline(display_frame, f"Error DX: {dx} DY: {dy}", (10, 30), (0, 0, 255))
                put_text_outline(display_frame, "LOCKED AND TRACKING", (10, 60), (0, 255, 255))

        # --- BUILD FINAL UI ---
        # Generate the bottom dashboard UI panel
        ui_panel = draw_hud(analyzer.data, current_step, state, focus_score, current_frame)
        
        # Combine the video feed and the HUD
        combined_frame = np.vstack((display_frame, ui_panel))
            
        cv2.imshow("Star-Track Vision System", combined_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    # ---------------------------------------------------------
    # CONFIGURATION FLAGS
    # ---------------------------------------------------------
    SIMULATE_VIDEO = True  
    SIMULATE_ARDUINO = True 
    
    VIDEO_SOURCE = str(SIM_VIDEO_DEFAULT) if SIMULATE_VIDEO else "ipc:///tmp/video_feed"
    # ---------------------------------------------------------

    main(SIMULATE_VIDEO, SIMULATE_ARDUINO, VIDEO_SOURCE)
