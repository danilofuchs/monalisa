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

anguloAtualServos = []
tempoUltimoComandoServos = []
limitesAngulo = ()
servos = []
resolucaoCaptura = ()
capturaVideo = 0
cameraIndex = 0
board = 0

def globals() :
    global anguloAtualServos
    anguloAtualServos = [0, 0]
    global tempoUltimoComandoServos
    tempoUltimoComandoServos = [0, 0]
    #Limites de ângulo do servo (min, max)
    # min = olho totalmente na esquerda, max = direi
    global limitesAngulo
    limitesAngulo = (65, 115)

    servoPin1 = 8
    servoPin2 = 9
    servo1 = board.get_pin('d:{0}:s'.format(servoPin1))
    servo2 = board.get_pin('d:{0}:s'.format(servoPin2))
    global servos
    servos = [servo1, servo2]

    global cameraIndex
    cameraIndex = 0
    global resolucaoCaptura
    resolucaoCaptura = [400, 300]
    global capturaVideo
    capturaVideo = cv2.VideoCapture(cameraIndex)
    capturaVideo.set(cv2.CAP_PROP_FRAME_WIDTH, resolucaoCaptura[0])
    capturaVideo.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucaoCaptura[1])

    #Load a cascade file for detecting faces
    global face_cascade
    face_cascade = cv2.CascadeClassifier('facial_recognition_model.xml')

def initArduino() :
    global board
    sucessoArduino = False
    for i in range (0, 10) :
        try:
            board = Arduino('/dev/ttyUSB{0}'.format(i))
            sucessoArduino = True
            break
        except serial.serialutil.SerialException as e:
            continue
    if sucessoArduino :
        print('Arduino conectado')
    else :
        print('Erro ao conectar com o arduino!')
        exit()
    
def setAngulo(index, angulo):
    #print('Servo em {0}'.format(int(angulo)))
    servos[index].write(int(angulo))
    anguloAtualServos[index] = int(angulo)
    tempoUltimoComandoServos[index] = time.time()

def detectarFaces(image, face_cascade) :

    #Converte imagem para preto/branco (3x mais rapido)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Encontra faces pelo modelo treinado
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    #Encontrou faces?
    if len(faces) > 0 : 
        return faces
    else :
        raise LookupError('No face found')

def faceEstaNoVetor(faces, faceAlvo) :
    found = False
    for face in faces:
        if (face == faceAlvo).all() :
            found = True
    
    return found

def acharMaiorFace(faces, facesParaIgnorar) :
    maiorFace = (0,0,0,0)
    found = False
    for face in faces:
        if not faceEstaNoVetor(facesParaIgnorar, face) :
            (x, y, w, h) = face
            if w * h > maiorFace[2] * maiorFace[3] :
                found = True
                maiorFace = (x,y,w,h)
    return maiorFace

def desenharResultado(image, faces, linhasServos) :

    for (x,y,w,h) in faces :
        if (x,y,w,h) != (0,0,0,0) :
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)

    for (x,y,w,h) in linhasServos :
        if (x,y,w,h) != (0,0,0,0) :
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,255),2)

    cv2.imshow('result', cv2.resize(image, (640,480)))
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

def limitarValor(valor, intervalo) :
    return max(min(valor, intervalo[1]), intervalo[0])

def limiteMovimento(servoIndex) :
    amplitudeServos = limitesAngulo[1] - limitesAngulo[0]
    limite = 0.03 * amplitudeServos
    return limite

def seguirFace(servoIndex, face, image) :
    (x, y, w, h) = face

    posicaoMedia = x + (w/2)
    posicaoRelativaFoto = posicaoMedia / (image.shape[0] * 1.0)

    minAngulo = limitesAngulo[0]
    maxAngulo = limitesAngulo[1]

    angulo = limitarValor(maxAngulo - (maxAngulo - minAngulo) * posicaoRelativaFoto, limitesAngulo)
    distanciaMovimento = abs(angulo - anguloAtualServos[servoIndex])
    if distanciaMovimento > limiteMovimento :
        angulo = limiteMovimento

    setAngulo(servoIndex, angulo)

    return posicaoRelativaFoto

def moverBaseadoNasFaces(faces, image) :
    anguloServosRelativo = (0, 0)
    facesDecrescente = faces

    facesParaIgnorar = []

    i = 0
    for (x, y, w, h) in faces:
        facesDecrescente[i] = acharMaiorFace(faces, facesParaIgnorar)
        facesParaIgnorar.append(facesDecrescente[i])
        i += 1

    numeroFaces = facesDecrescente.shape[0]

    anguloServosRelativo = [0, 0]

    if numeroFaces == 0 :
        a = 0
    elif numeroFaces == 1 :
        anguloServosRelativo[0] = seguirFace(0, facesDecrescente[0], image)
        anguloServosRelativo[1] = seguirFace(1, facesDecrescente[0], image)
    elif numeroFaces == 2 :
        anguloServosRelativo[0] = seguirFace(0, facesDecrescente[0], image)
        anguloServosRelativo[1] = seguirFace(1, facesDecrescente[1], image)

    return anguloServosRelativo

def tocarSom(nomeArquivo) :
    pygame.mixer.music.load(nomeArquivo)
    pygame.mixer.music.play()

def somInicial() :
    tocarSom('audio/start_race.mp3')

''' INICIO '''

pygame.init()
initArduino()
globals()

somInicial()

for i in range(limitesAngulo[0], limitesAngulo[1]) :
    setAngulo(0, i) 
    setAngulo(1, i) 
    time.sleep(0.005) 
for i in range(limitesAngulo[1], limitesAngulo[0], -1) :
    setAngulo(0, i) 
    setAngulo(1, i) 
    time.sleep(0.005) 
start = time.time()
i = 0

while time.time() - start < 240 and capturaVideo.isOpened():
    
    ret, frame = capturaVideo.read()
    cv2.resize(frame, (320,240))
    temFacesNaImagem = True
    try :
        faces = detectarFaces(frame, face_cascade)
    except LookupError :
        temFacesNaImagem = False
    if temFacesNaImagem :
        anguloServosRelativo = moverBaseadoNasFaces(faces, frame)
        #setAngle(servo2, anguloServosRelativo[0] * 100) 
        linhasServos = []
        for angulo in anguloServosRelativo :
            rectangle = (int((frame.shape[0]) * angulo), 0, 1, frame.shape[1])
                        #(x, y, w, h)
            linhasServos.append(rectangle)
        desenharResultado(frame, faces, linhasServos)
    else :
        desenharResultado(frame, np.array([(0,0,0,0)]), np.array([(0,0,0,0)]))
    
    i = i + 1
    current = time.time()
    print('Processed {0} frames in {1:.2f}s. Avg {2:.2f}fps\r'.format(i, (current-start), i/(current-start), end='\r'))
    
capturaVideo.release()
end = time.time()
print('Elapsed time: ' + str(end-start) + 's')