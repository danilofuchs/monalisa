import serial
import time


for i in range (0,10) :
    try:
        ser = serial.Serial(
                
                port='/dev/ttyUSB' + str(i),
                baudrate = 9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
        break
    except serial.serialutil.SerialException:
        continue



ser.write(b's1 0')
print(ser.readline())
time.sleep(1)
ser.write(b's1 z')
print(ser.readline())

for i in range(24, 60) :
    print("sending \"{0}\"".format('s0 ' + chr(i*2)))
    string = 's0 ' + chr(i) + '\n'
    ser.write(string.encode())
    print(ser.readline())
    time.sleep(0.04)

#ser.write(b's1 a\n')
#ser.write('s0   \n')

ser.close()