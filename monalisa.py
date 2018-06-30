# -*- coding: utf-8 -*-

''' CROSSBOTS WINTER CHALLENGE 2018
''          MONALISA
''  DESC: Reconhece uma face e move olhos (servo)
''        para seguir a face
'''
import cv2
import numpy as np

import time

from pyfirmata import Arduino, util
import serial

import pygame

captureRes = (400, 300)

#Load a cascade file for detecting faces
face_cascade = cv2.CascadeClassifier('facial_recognition_model.xml')
cameraIndex = 0

video_capture = cv2.VideoCapture(cameraIndex)

successArduino = False

for i in range (0,10) :
    try:
        board = Arduino('/dev/ttyUSB{0}'.format(i))
        successArduino = True
        break
    except serial.serialutil.SerialException as e:
        continue
if successArduino :
    print('Arduino conectado')
else :
    print('Erro ao conectar com o arduino!')
    exit()
    

servoPin1 = 8
servoPin2 = 9
servo1 = board.get_pin('d:{0}:s'.format(servoPin1))
servo2 = board.get_pin('d:{0}:s'.format(servoPin2))

#Limites de ângulo do servo (min, max)
# min = olho totalmente na esquerda, max = direita
angleLimits = (65, 115)

def setAngle(servo, angle):
    print('Servo em {0}'.format(int(angle)))
    servo.write(int(angle))

def detectFaces(image, face_cascade) :

    #Converte imagem para preto/branco (3x mais rapido)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Encontra faces pelo modelo treinado
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    #Encontrou faces?
    
    if len(faces) > 0 : 
        return faces
    else :
        raise LookupError('No face found')

def isFaceInArray(faces, targetFace) :
    found = False
    for face in faces:
        #print("{0}, {1}".format(type(face), type(targetFace)))
        if (face == targetFace).all() :
            found = True
    
    return found

def findBiggestFace(faces, ignoredFaces) :
    biggestFace = (0,0,0,0)
    found = False
    for face in faces:
        if not isFaceInArray(ignoredFaces, face) :
            (x, y, w, h) = face
            if w * h > biggestFace[2] * biggestFace[3] :
                found = True
                biggestFace = (x,y,w,h)
    return biggestFace

def drawResult(image, faces, servoRectangles) :

    for (x,y,w,h) in faces :
        if (x,y,w,h) != (0,0,0,0) :
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)

    for (x,y,w,h) in servoRectangles :
        if (x,y,w,h) != (0,0,0,0) :
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,255),2)

    cv2.imshow('result', cv2.resize(image, (640,480)))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()


def moveBasedOnFaces(faces, image, angleLimits, servoArray) :
    anguloServosRelativo = (0, 0)
    facesDecrescent = faces

    ignoredFaces = []
    #ignoredFaces = np.zeros((faces.shape[0], 1), dtype=tuple)
    i = 0
    for (x, y, w, h) in faces:
        facesDecrescent[i] = findBiggestFace(faces, ignoredFaces)
        ignoredFaces.append(facesDecrescent[i])
        i += 1

    (x, y, w, h) = facesDecrescent[0]
    if x != 0 :

        posicaoMedia = x + (w/2)
        posicaoRelativaFoto = posicaoMedia / (image.shape[0] * 1.0)

        minAngulo = angleLimits[0]
        maxAngulo = angleLimits[1]

        angulo = min(max(maxAngulo - (maxAngulo - minAngulo) * posicaoRelativaFoto, angleLimits[0]), angleLimits[1])

        setAngle(servoArray[0], angulo)
        setAngle(servoArray[1], angulo)

        anguloServosRelativo = (posicaoRelativaFoto, posicaoRelativaFoto)

    return anguloServosRelativo

def tocarSom(nomeArquivo) :
    pygame.mixer.music.load(nomeArquivo)
    pygame.mixer.music.play()

def initialSound() :
    tocarSom('audio/start_race.mp3')

video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, captureRes[0])
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, captureRes[1])

initialSound()
for i in range(angleLimits[0], angleLimits[1]) :
    setAngle(servo1, i) 
    setAngle(servo2, i) 
    time.sleep(0.005) 
for i in range(angleLimits[1], angleLimits[0], -1) :
    setAngle(servo1, i) 
    setAngle(servo2, i) 
    time.sleep(0.005) 
start = time.time()
i = 0

while time.time() - start < 240 and video_capture.isOpened():
    
    ret, frame = video_capture.read()
    cv2.resize(frame, (320,240))
    thereAreFacesInImage = True
    try :
        faces = detectFaces(frame, face_cascade)
    except LookupError :
        thereAreFacesInImage = False
    if thereAreFacesInImage :
        anguloServosRelativo = moveBasedOnFaces(faces, frame, angleLimits, [servo1, servo2])
        #setAngle(servo2, anguloServosRelativo[0] * 100) 
        angleLines = []
        for angle in anguloServosRelativo :
            rectangle = (int((frame.shape[0]) * angle), 0, 1, frame.shape[1])
                        #(x, y, w, h)
            angleLines.append(rectangle)
        drawResult(frame, faces, angleLines)
    else :
        drawResult(frame, np.array([(0,0,0,0)]), np.array([(0,0,0,0)]))
    
    i = i + 1
    current = time.time()
    print('Processed {0} frames in {1:.2f}s. Avg {2:.2f}fps\r'.format(i, (current-start), i/(current-start), end='\r'))
    
video_capture.release()
#out.release()
end = time.time()
print('Elapsed time: ' + str(end-start) + 's')