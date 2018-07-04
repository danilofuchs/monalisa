# -*- coding: utf-8 -*-

''' CROSSBOTS WINTER CHALLENGE 2018
''          MONALISA
''  DESC: Reconhece uma face e move olhos (servo)
''        para seguir a face
'''
import cv2
import numpy as np

import time
from random import randint

from pyfirmata import Arduino, util
import serial

import pygame


'''      MODOS DIFERENTES DE REAGIR            ''
''  0 - Seguir apenas 1 pessoa                 ''
''  1 - Seguir 2 pessoas cada uma em 1 olho    ''
''  2 - Esconder os olhos - Não seguir ninguém ''
''  3 - Louca                                  '''

limiteTempo = 0
anguloAtualServos = []
tempoUltimoComandoServos = []
limitesAngulo = []
velocidadeMaximaServos = 0
servos = []

tempoUltimaDeteccao = 0
resolucaoCaptura = ()
capturaVideo = 0
cameraIndex = 0
board = 0

sonsRecentes = []
tempoAntesDeRepetirSons = 0

def globals() :
    global limiteTempo
    limiteTempo = 10000
    global anguloAtualServos
    anguloAtualServos = [0, 0]
    global tempoUltimoComandoServos
    tempoUltimoComandoServos = [0, 0]
    #Limites de ângulo do servo (min, max)
    # min = olho totalmente na esquerda, max = direi
    global limitesAngulo
    limitesAngulo.append((60, 105))
    #limitesAngulo.append((65, 115))
    limitesAngulo.append((45, 80))
    #Velocidade máxima de reotação em graus / segundo
    global velocidadeMaximaServos
    velocidadeMaximaServos = 40

    servoPin1 = 8
    servoPin2 = 9
    servo1 = board.get_pin('d:{0}:s'.format(servoPin1))
    servo2 = board.get_pin('d:{0}:s'.format(servoPin2))
    global servos
    servos = [servo1, servo2]

    global tempoUltimaDeteccao
    tempoUltimaDeteccao = 0
    global cameraIndex
    cameraIndex = 1
    global resolucaoCaptura
    resolucaoCaptura = [400, 300]
    global capturaVideo
    capturaVideo = cv2.VideoCapture(cameraIndex)
    capturaVideo.set(cv2.CAP_PROP_FRAME_WIDTH, resolucaoCaptura[0])
    capturaVideo.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucaoCaptura[1])

    #Load a cascade file for detecting faces
    global face_cascade
    face_cascade = cv2.CascadeClassifier('facial_recognition_model.xml')

    global sonsRecentes
    sonsRecentes = []
    global tempoAntesDeRepetirSons
    tempoAntesDeRepetirSons = 120

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
    angulo = int(angulo)
    servos[index].write(angulo)
    anguloAtualServos[index] = int(angulo)
    tempoUltimoComandoServos[index] = time.time()
    
def tocarSom(nomeArquivo) :
    if not pygame.mixer.music.get_busy() and not somFoiTocadoRecente(nomeArquivo, tempoAntesDeRepetirSons) :
        pygame.mixer.music.load(nomeArquivo)
        pygame.mixer.music.play()
        sonsRecentes.append((nomeArquivo, time.time()))

def somFoiTocadoRecente(nomeArquivo, tempoMaximo=120) :
    foiTocado = False
    for (som, tempo) in sonsRecentes :
        if tempo > (time.time() - tempoMaximo) :
            if som == nomeArquivo :
                foiTocado = True
                break
    return foiTocado

def somDoTipoFoiTocadoRecente(prefixoSom, tempoMaximo=120) :
    foiTocado = False
    for i in range(1, 10) :
        if somFoiTocadoRecente('audio/{0}{1}.mp3'.format(prefixoSom, i), tempoMaximo) :
            foiTocado = True
            break
    return foiTocado

def removerSonsAntigos() :
    for (som, tempo) in sonsRecentes :
        if time.time() - tempo > 120 :
            sonsRecentes.remove((som, tempo))

def somInicial() :
    tocarSom('audio/start_race.mp3')

def detectarFaces(image, face_cascade) :

    #Converte imagem para preto/branco (3x mais rapido)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #Encontra faces pelo modelo treinado
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    #Encontrou faces?
    if len(faces) > 0 : 
        #if not pygame.mixer.music.get_busy():
        #    tocarSom('audio/oi_gato.mp3')
        tempoUltimaDeteccao = time.time()
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

def limitarValor(valor, intervalo: tuple) :
    return max(min(valor, intervalo[1]), intervalo[0])

def limiteMovimento(servoIndex) :
    #amplitudeServos = limitesAngulo[1] - limitesAngulo[0]
    tempoDecorrido = time.time() - tempoUltimoComandoServos[servoIndex]
    #velocidade = amplitudeServos / tempoDecorrido
    #limite = limitarValor(velocidade, (0, velocidadeMaximaServos))

    limitePorVelocidade = velocidadeMaximaServos * tempoDecorrido
    limitePorDistancia = velocidadeMaximaServos / 5

    limite = min(limitePorDistancia, limitePorVelocidade)
    return limite

def seguirFace(servoIndex, face, image) :
    (x, y, w, h) = face

    posicaoMedia = x + (w/2)
    posicaoRelativaFoto = posicaoMedia / (image.shape[0] * 1.0)

    minAngulo = limitesAngulo[servoIndex][0]
    maxAngulo = limitesAngulo[servoIndex][1]

    angulo = limitarValor(maxAngulo - (maxAngulo - minAngulo) * posicaoRelativaFoto, limitesAngulo[servoIndex])
    distanciaMovimento = angulo - anguloAtualServos[servoIndex]
    limite = limiteMovimento(servoIndex)
    if distanciaMovimento > limite :
        angulo = anguloAtualServos[servoIndex] + limite
    elif distanciaMovimento < -limite :
        angulo = anguloAtualServos[servoIndex] - limite        

    setAngulo(servoIndex, angulo)

    return posicaoRelativaFoto

def posicaoRelativaParaAngulo(posicaoRelativa, indexServo, image) :
    amplitudeServo = limitesAngulo[indexServo][1] - limitesAngulo[indexServo][0]
    angulo = posicaoRelativa * amplitudeServo + limitesAngulo[indexServo][0]
    return angulo

def anguloParaPosicaoRelativa(angulo, indexServo) :
    amplitudeServo = limitesAngulo[indexServo][1] - limitesAngulo[indexServo][0]
    posicaoRelativa = (angulo - limitesAngulo[indexServo][1] ) / - amplitudeServo
    return posicaoRelativa

def modoLouco() :
    tocarSom('audio/grito1.mp3')
    amplitudeServo0 = limitesAngulo[0][1] - limitesAngulo[0][0]
    amplitudeServo1 = limitesAngulo[1][1] - limitesAngulo[1][0]

    for i in range(0, 30) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(30, 15, -1) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(15, 30) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(30, 0, -1) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(0, 30) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        time.sleep(0.01)
    for i in range(0, 30) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(30, 15, -1) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(15, 30) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(30, 0, -1) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)
    for i in range(0, 15) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.01)

def modoSeguirDuas(faces, image) :
    a = 0

def modoEsconderOlhos() :
    while ( anguloAtualServos[0] > limitesAngulo[0][0] - 10 and
            anguloAtualServos[1] < limitesAngulo[1][1] + 10) :
        setAngulo(0, anguloAtualServos[0] - 5)
        setAngulo(1, anguloAtualServos[1] + 5)
        time.sleep(0.05)


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
    (x,y,w,h) = facesDecrescente[0]
    if x == 0 :
        numeroFaces = 0

    anguloServosRelativo = [0, 0]


    if numeroFaces == 0 :
        if time.time() - tempoUltimaDeteccao > 10 :
            (x, y, w, h) = (image.shape[0]/2, 0, 2, 2)
            anguloServosRelativo[0] = seguirFace(0, (x, y, w, h), image)
            anguloServosRelativo[1] = seguirFace(1, (x, y, w, h), image)
        elif time.time() - tempoUltimaDeteccao > 60 :
            if randint(0, 1000) < 10 :
                if not somFoiTocadoRecente('semmovimento', 240) :
                    tocarSom('audio/semmovimento1.mp3')
        else :
            if randint(0, 10000) == 1 :
                print('louco')
                modoLouco()
            else :
                anguloServosRelativo[0] = anguloParaPosicaoRelativa(anguloAtualServos[0], 0)
                anguloServosRelativo[1] = anguloParaPosicaoRelativa(anguloAtualServos[1], 1)
    else :
        if time.time() - tempoUltimaDeteccao > 3 :
            if not somDoTipoFoiTocadoRecente('ola', 30) :
                somNum = randint(1,3)
                tocarSom('audio/ola{0}.mp3'.format(somNum))
            if randint(0, 1000) < 5 :
                if not somDoTipoFoiTocadoRecente('cantada', 120) :
                    tocarSom('audio/cantada1.mp3')
        if numeroFaces == 1 :
            anguloServosRelativo[0] = seguirFace(0, facesDecrescente[0], image)
            anguloServosRelativo[1] = seguirFace(1, facesDecrescente[0], image)
        elif numeroFaces == 2 :
            anguloServosRelativo[0] = seguirFace(0, facesDecrescente[0], image)
            anguloServosRelativo[1] = seguirFace(1, facesDecrescente[1], image)
        else :
            #muita gente
            if randint(0, 100) < 75 :
                #75% de chance de dizer que está nervosa
                somNum = randint(1,3)
                tocarSom('audio/muitaspessoas{0}.mp3'.format(somNum))
                if randint(0, 100) < 50 :
                    modoEsconderOlhos()

    aleatorio = randint(0, 8000)
    if aleatorio < 30 :
        if aleatorio < 15 :
            somNum = randint(1,8)
            tocarSom('audio/aleatorio{0}.mp3'.format(somNum))
        elif aleatorio < 20 :
            if not somDoTipoFoiTocadoRecente('passa', 60) :
                somNum = randint(1,3)
                tocarSom('audio/passa{0}.mp3'.format(somNum))
        else :
            if not somDoTipoFoiTocadoRecente('foto', 60) :
                somNum = randint(1,5)
                tocarSom('audio/foto{0}.mp3'.format(somNum))


    return anguloServosRelativo

def movimentoInicial() :
    amplitudeServo0 = limitesAngulo[0][1] - limitesAngulo[0][0]
    amplitudeServo1 = limitesAngulo[1][1] - limitesAngulo[1][0]
    for i in range(0, 30) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.02)
    for i in range(30, 0, -1) :
        setAngulo(0, (i * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.02)

''' INICIO '''


pygame.init()
initArduino()
globals()

print(anguloParaPosicaoRelativa(115, 0))

somInicial()
movimentoInicial()

start = time.time()
i = 0

'''
j = 0
while pygame.mixer.music.get_busy() :
    j += 1
print('inicio')
modoLouco()
print('fim')
'''

while time.time() - start < limiteTempo and capturaVideo.isOpened():
    ret, frame = capturaVideo.read()
    cv2.resize(frame, (320,240))
    temFacesNaImagem = True
    try :
        faces = detectarFaces(frame, face_cascade)
    except LookupError :
        faces = np.array([(0,0,0,0)])
        temFacesNaImagem = False

    anguloServosRelativo = moverBaseadoNasFaces(faces, frame)
    linhasServos = []
    for angulo in anguloServosRelativo :
        rectangle = (int((frame.shape[0]) * angulo), 0, 1, frame.shape[1])
                    #(x, y, w, h)
        linhasServos.append(rectangle)
    desenharResultado(frame, faces, linhasServos)
    
    #removerSonsAntigos()

    i = i + 1
    current = time.time()
    print('Processed {0} frames in {1:.2f}s. Avg {2:.2f}fps\r'.format(i, (current-start), i/(current-start), end='\r'))
    
capturaVideo.release()
end = time.time()
print('Elapsed time: ' + str(end-start) + 's')