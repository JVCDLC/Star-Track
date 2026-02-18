import serial
import time
import threading
from queue import Queue
from enum import Enum
import msvcrt


class Motor(Enum):
    DECLIMATION = 1
    RIGHT_ASCENSION = 2
    FOCUS = 3


rx_queue = Queue()
running = True
speed_values = [0.00833, 0.05, 0.1, 0.5, 1, 3]
speed_index = 1  # start at 0.1


def serial_reader(ser):
    while running:
        if ser.in_waiting:
            reply = ser.readline().decode(errors="ignore").strip()
            if reply:
                print("RX:", reply)
                rx_queue.put(reply)
        else:
            time.sleep(0.001)

def send_request_motor(ser, motor: Motor, position: float):
    message = f"REQUEST,{motor.name},{position}\n"
    ser.write(message.encode())
    #print("TX:", message.strip())

### send motor and angle to esp-32.  wait for the answer back from the esp32
#   REQUEST,MOTOR,180.1234\n
def send_request_motor2(ser, motor: Motor, position: float, timeout=1.0) -> bool:
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
ser = serial.Serial('COM8', 115200, timeout=0)
time.sleep(2)  # wait for esp-32 to reset

reader_thread = threading.Thread(target=serial_reader, args=(ser,), daemon=True)
reader_thread.start()

ra_pos = 0.0
dec_pos = 0.0

speed_factor = speed_values[speed_index]



try:
    target=0.0
    while True:

        if msvcrt.kbhit():
            key = msvcrt.getch().lower()
            #print("key pressed:", key)

            # Test with key control
            if key == b'w':
                dec_pos += speed_factor
                send_request_motor(ser, Motor.DECLIMATION, dec_pos)

            elif key == b's':
                dec_pos -= speed_factor
                send_request_motor(ser, Motor.DECLIMATION, dec_pos)

            elif key == b'a':
                ra_pos -= speed_factor
                send_request_motor(ser, Motor.RIGHT_ASCENSION, ra_pos)

            elif key == b'd':
                ra_pos += speed_factor
                send_request_motor(ser, Motor.RIGHT_ASCENSION, ra_pos)

            elif key == b'\xe0':
                arrow = msvcrt.getch()
                if arrow == b'H':  # up arrow
                    if speed_index < len(speed_values) - 1:
                        speed_index += 1
                        speed_factor = speed_values[speed_index]
                    print("Speed:", speed_factor)

                elif arrow == b'P':  # down arrow
                    if speed_index > 0:
                        speed_index -= 1
                        speed_factor = speed_values[speed_index]
                    print("Speed:", speed_factor)

        time.sleep(0.01)

    """
    while True:
        # send_request_motor(ser, Motor.RIGHT_ASCENSION, target)
        # time.sleep(1)
        # target += 0.0416
        #
        # 120.15 degrees
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 45)
        send_request_motor(ser, Motor.DECLIMATION, 45)
        time.sleep(5)

        # 0 degres
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 0.0)
        send_request_motor(ser, Motor.DECLIMATION, 0.0)
        time.sleep(5)
        """
# stop if user touch a key
except KeyboardInterrupt:
    print("stop")

finally:
    running = False
    time.sleep(0.1)
    ser.close()
