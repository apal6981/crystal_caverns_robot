import serial

try:
    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)
except Exception:
    print("can't open serial")
    exit(1)
print("serial initialized")

while ser.is_open:
    if ser.in_waiting > 21:
        print(ser.readline().decode().rstrip().split(" "))

print("serial closing")
ser.close()
