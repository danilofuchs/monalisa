from pyfirmata import Arduino, util
import serial
import time

for i in range (0,10) :
    try:
        board = Arduino('/dev/ttyUSB{0}'.format(i))
        break
    except serial.serialutil.SerialException:
        continue

servo1 = board.get_pin('d:8:s')
servo2 = board.get_pin('d:9:s')

def move_servo(servo, a):
    servo.write(a)


while True:
    for i in range(0, 180) :
        move_servo(servo1, i) 
        move_servo(servo2, i) 
        time.sleep(0.01) 

    for i in range(180, 0, -1) :
        move_servo(servo1, i) 
        move_servo(servo2, i) 
        time.sleep(0.01) 


board.exit()