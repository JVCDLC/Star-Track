from asyncio import wait

import serial
import time
from enum import Enum


class Motor(Enum):
    DECLIMATION = 1
    RIGHT_ASCENSION = 2
    FOCUS = 3

### send motor and angle to esp-32.  wait for the answer back from the esp32
#   REQUEST,MOTOR,180.1234\n
def send_request_motor(ser, motor: Motor, position: float, timeout=1.0) -> bool:
    message = f"REQUEST,{motor.name},{position}\n"
    ser.write(message.encode())
    print("TX:", message.strip())

    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:

            # Succes if we ave a answer
            reply = ser.readline().decode().strip()
            while not(reply.startswith("ACK")):
                reply = ser.readline().decode().strip()
                print("RX:", reply)
                time.sleep(0.001)
            return True


    # timeout took too much time to have answer, should implement: send a new message if no answer
    print("RX: timeout")
    return False


# For Linux(Raspberry pi)
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # wait for esp-32 to reset

try:
    while True:
        # 120.15 degrees
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 0.45)
        time.sleep(1)

        # 0 degres
        send_request_motor(ser, Motor.RIGHT_ASCENSION, 0.0)
        time.sleep(1)

# stop if user touch a key
except KeyboardInterrupt:
    print("stop")

finally:
    ser.close()


