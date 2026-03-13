import serial
import time

ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)

ser.write(b'AT\r')

time.sleep(1)

data = ser.read(100)

print("RX:",data)

ser.close()