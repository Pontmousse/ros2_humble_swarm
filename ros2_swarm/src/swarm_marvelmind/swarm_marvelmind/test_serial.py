import serial

# Open serial port where beacon is connected
ser = serial.Serial('/dev/ttyACM0', baudrate=500000, timeout=1)

while True:
    data = ser.read(32)  # Read raw binary data
    print(data)
