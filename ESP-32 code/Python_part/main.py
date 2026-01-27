import serial
import time
# pour linux(Raspberry pi)
#ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)  # important pour laisser l'ESP32 reset

ser.write(b'hello esp32\n')

reply = ser.readline().decode().strip()
print(reply)
