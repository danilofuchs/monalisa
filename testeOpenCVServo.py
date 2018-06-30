#Controle de servo no Raspberry Pi
#Autor: Simon Monk
import cv2
import numpy as np
 
import RPi.GPIO as GPIO
import time

servo_pin = 21

#Ajuste estes valores para obter o intervalo completo do movimento do servo
deg_0_pulse   = 0.5 
deg_180_pulse = 2.5
f = 50.0

# Faca alguns calculos dos parametros da largura do pulso
period = 1000/f
k      = 100/period
deg_0_duty = deg_0_pulse*k
pulse_range = deg_180_pulse - deg_0_pulse
duty_range = pulse_range * k

#Iniciar o pino gpio
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
pwm = GPIO.PWM(servo_pin,f)
pwm.start(0)
 
def set_angle(angle):
    duty = deg_0_duty + (angle/180.0)* duty_range
    pwm.ChangeDutyCycle(duty)

def detectFace(image) :

    #Load a cascade file for detecting faces
    face_cascade = cv2.CascadeClassifier('facial_recognition_model.xml')
    #Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Look for faces in the image using the loaded cascade file
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    biggestFace = (0,0,0,0)
    for (x,y,w,h) in faces:
        if w * h > biggestFace[2] * biggestFace[3] :
            biggestFace = (x,y,w,h)

    (x,y,w,h) = biggestFace

    if x != 0 :
        cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)

        posicaoMedia = x + (w/2)
        posicaoRelativaFoto = posicaoMedia / (image.shape[0] * 1.0)

        cv2.rectangle(image,(posicaoMedia,0),(posicaoMedia + 2,image.shape[1]),(255,0,255),2)

        minAngulo = 60
        maxAngulo = 120

        angulo = maxAngulo - (maxAngulo - minAngulo) * posicaoRelativaFoto
        
        print('Movido para o angulo {0:.1f}graus'.format(angulo))
        set_angle(angulo)

    #cv2.imshow('result', cv2.resize(image, (1000,700)))
    #cv2.waitKey()
    return image
'''
try :
    image = cv2.imread('samples/image3.1.jpg', flags=cv2.IMREAD_COLOR)
    detectFace(image)
except cv2.error as e :
    print(e)
'''
if True :
    start = time.time()
    video_capture = cv2.VideoCapture(0)
    #video_capture.set(cv2.CAP_PROP_BRIGHTNESS, 1)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 200)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 150)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('outputCamera.avi',fourcc, 1.0, (640,360))

    i = 0
    while time.time() - start < 30 and video_capture.isOpened():
        ret, frame = video_capture.read()
        '''
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = 1
        hsv[:,:,2] += 10
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        '''
        image = detectFace(frame)
        
        #cv2.imwrite('output/result{0}.jpg'.format(i),image)
        #out.write(image)
        i = i + 1
        current = time.time()
        print('Processed {0} frames in {1:.2f}s. Avg {2:.2f}fps'.format(i, (current-start), i/(current-start), end='\r'))
        
        cv2.imshow('result', cv2.resize(image, (800,600)))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    #out.release()
    end = time.time()
    print('Elapsed time: ' + str(end-start) + 's')