import serial

ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1)

# frame enable servo
frame = bytes.fromhex("01 00 1A 00 00 01 00 00 00")

ser.write(frame)

data = ser.read(32)

print("RX:",data)

ser.close()