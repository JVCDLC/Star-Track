import serial
import time
import threading
from queue import Queue
from enum import Enum


class Motor(Enum):
    DECLIMATION = 1
    RIGHT_ASCENSION = 2
    FOCUS = 3


rx_queue = Queue()
running = True


def serial_reader(ser):
    while running:
        if ser.in_waiting:
            reply = ser.readline().decode(errors="ignore").strip()
            if reply:
                print("RX:", reply)
                rx_queue.put(reply)
        else:
            time.sleep(0.001)


### send motor and angle to esp-32.  wait for the answer back from the esp32
#   REQUEST,MOTOR,180.1234\n
def send_request_motor(ser, motor: Motor, position: float, timeout=1.0) -> bool:
    message = f"REQUEST,{motor.name},{position}\n"
    ser.write(message.encode())
    print("TX:", message.strip())

    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            reply = rx_queue.get(timeout=0.05)
            if reply.startswith("ACK"):
                return True
        except:
            pass

    # timeout took too much time to have answer, should implement: send a new message if no answer
    print("RX: timeout")
    return False


# For Linux(Raspberry pi)
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
ser = serial.Serial('COM5', 115200, timeout=0)
time.sleep(2)  # wait for esp-32 to reset

reader_thread = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
reader_thread.start()

try:
    while True:
        # 120.15 degrees
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 180)
        time.sleep(5)

        # 0 degres
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 0.0)
        time.sleep(5)

# stop if user touch a key
except KeyboardInterrupt:
    print("stop")

finally:
    running = False
    time.sleep(0.1)
    ser.close()
