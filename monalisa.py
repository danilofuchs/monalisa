# -*- coding: utf-8 -*-
from sys import exit
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

import json
import gevent
import grequests

import argparse
parser = argparse.ArgumentParser(description='Reconhecimento de faces com movimento de olhos e fala.')
parser.add_argument('--no-arduino', action='store_true',
                   help='Desabilita a comunicação com o arduino e servos')
parser.add_argument('--no-facebook', action='store_true',
                   help='Desabilita a interação com likes na página')
parser.add_argument('--no-geek', action='store_true',
                   help='Desabilita as frases especiais da geek city')
parser.add_argument('--processo', action='store_true',
                   help='Habilita frases sobre o processo seletivo')
args = parser.parse_args()

'''      MODOS DIFERENTES DE REAGIR                                         ''
''  0 - Seguir apenas 1 pessoa                                              ''
''  1 - Seguir 2 pessoas cada uma em 1 olho (maior em 1, menor em outro)    ''
''  2 - Seguir 2 pessoas cada uma em 1 olho (esq = esq, dir = dir)          ''
''  3 - Esconder os olhos - Não seguir ninguém                              ''
''  4 - Louca                                                               '''


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

DEFAULT_MULTIPLICADOR_INTERVALO = 3
multiplicadorIntervalo = DEFAULT_MULTIPLICADOR_INTERVALO

ordemMedia = 0
posicaoRelativaAtualServos = []

# ID página crossbots
PAGE_ID = "crossbotsutfpr" 
# Token de acesso (Expira a cada 2 meses. Contatar administrador da página para liberar acesso)
ACCESS_TOKEN = "EAAfs9OJXRZB8BAAyYCgELf2sOZC46zp7ZAoQ7dOcTzO4wLYQ6UVK8hmAMzAsZBCG2fEZAmHFKKkMHXlmO6ACjR1O5TowZAPZBqc187D38TjQKKiCkRmFBTehPYuwXgeZByMySH0s0qc3U0ZBRvRyQ5TGOB2DNzLMYUM25SAOmUPT2ZBgZDZD"
tempoUltimosDadosFacebook = 0
likesFacebook = 0

def globals() :
    global limiteTempo
    limiteTempo = 20000
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
   # velocidadeMaximaServos = 40
    velocidadeMaximaServos = 30
    
    global args
    if (not args.no_arduino):
        servoPin1 = 8
        servoPin2 = 9
        servo1 = board.get_pin('d:{0}:s'.format(servoPin1))
        servo2 = board.get_pin('d:{0}:s'.format(servoPin2))
        global servos
        servos = [servo1, servo2]
    

    global tempoUltimaDeteccao
    tempoUltimaDeteccao = 0
    global cameraIndex
    cameraIndex = args.camera_index
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

    global ordemMedia
    ordemMedia = 5
    global posicaoRelativaAtualServos
    posicaoRelativaAtualServos = [0,0]

def initArduino() :
    global args
    if (not args.no_arduino):
        global board
        sucessoArduino = False
        for i in range (0, 10) :
            try:
                #board = Arduino('/dev/ttyUSB{0}'.format(i))
                board = Arduino('COM{0}'.format(i))
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
    global servos
    global anguloAtualServos
    global tempoUltimoComandoServos
    angulo = int(angulo)
    global args
    if (not args.no_arduino):
        servos[index].write(angulo)
    anguloAtualServos[index] = int(angulo)
    tempoUltimoComandoServos[index] = time.time()

def requestDadosFacebook(page_id, access_token):
    api_endpoint = "https://graph.facebook.com/v2.4/"
    fb_graph_url = api_endpoint+page_id+"?fields=fan_count&access_token="+access_token
    req = grequests.get(fb_graph_url, hooks=dict(response=receberDadosFacebook))
    request = req.send()
    global tempoUltimosDadosFacebook
    tempoUltimosDadosFacebook = time.time()

def receberDadosFacebook(response, *args, **kwargs) :
    global likesFacebook
    page_data = json.loads(response.content)
    newLikeCount = page_data['fan_count']
    if newLikeCount > likesFacebook :
        likesFacebook = newLikeCount
        random = randint(0, 3)
        global start
        if time.time() - start > 20 :
            #Não toca na primeira iteração
            tocarSom('audio/like{0}.mp3'.format(random))

def tocarSom(nomeArquivo) :
    if not pygame.mixer.music.get_busy() and not somFoiTocadoRecente(nomeArquivo, tempoAntesDeRepetirSons) :
        pygame.mixer.music.load(nomeArquivo)
        pygame.mixer.music.play()
        sonsRecentes.append((nomeArquivo, time.time()))
        print('Tocando: {0}'.format(nomeArquivo))

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
    tocarSom('audio/ola1.mp3')

def detectarFaces(image, face_cascade) :

    #Converte imagem para preto/branco (3x mais rapido)
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    #Encontra faces pelo modelo treinado
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)

    #Encontrou faces?
    if len(faces) > 0 : 
        #if not pygame.mixer.music.get_busy():
        #    tocarSom('audio/oi_gato.mp3')
        global tempoUltimaDeteccao
        tempoUltimaDeteccao = time.time()
        return faces
    else :
        raise LookupError('No face found')

def desenharResultado(image, faces, anguloServos) :
    fatorAmpliacao = (  640 / image.shape[1],
                        480 / image.shape[0])
    image = cv2.resize(image, (640,480))

    linhasServos = []
    j = 0
    for angulo in anguloServos :
        anguloRelativo = anguloParaPosicaoRelativa(angulo, j)
        xImagem = int(anguloRelativo * image.shape[1])
        cv2.line(image, (xImagem,0), (xImagem, image.shape[0]), (255,0,255),2)
        j+=1

    for (x,y,w,h) in faces :
        if (x,y,w,h) != (0,0,0,0) :
            (x1,y1) = (int(x * fatorAmpliacao[0]), int(y * fatorAmpliacao[1]))
            (x2,y2) = (int((x+w) * fatorAmpliacao[0]), int((y+h) * fatorAmpliacao[1]))
            cv2.rectangle(image,(x1,y1),(x2,y2),(255,255,0),2)

    text ='aperte Q para fechar'
    textPos = (10,460)
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontSize = 0.6
    color = (255,255,255)
    thickness = 1

    cv2.putText(image, text, textPos, font, fontSize, color, thickness, cv2.LINE_AA)

    cv2.imshow('result', image)

    #cv2.imshow('result', )
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

def limitarValor(valor, intervalo: tuple) :
    return max(min(valor, intervalo[1]), intervalo[0])

def limiteMovimento(servoIndex) :
    global tempoUltimoComandoServos
    global velocidadeMaximaServos

    tempoDecorrido = time.time() - tempoUltimoComandoServos[servoIndex]

    limitePorVelocidade = velocidadeMaximaServos * tempoDecorrido
    limitePorDistancia = velocidadeMaximaServos / 5

    limite = min(limitePorDistancia, limitePorVelocidade)
    return limite

def seguirFace(servoIndex, face, image) :
    global limitesAngulo
    (x, y, w, h) = face

    posicaoMedia = x + (w/2)
    posicaoRelativaFoto = posicaoMedia / (image.shape[1] * 1.0)

    global posicaoRelativaAtualServos
    global ordemMedia

    posicaoRelativaAtualServos[servoIndex] = posicaoRelativaAtualServos[servoIndex] * ((ordemMedia-1)/ordemMedia) + (posicaoRelativaFoto) / ordemMedia
    
    minAngulo = limitesAngulo[servoIndex][0]
    maxAngulo = limitesAngulo[servoIndex][1]

    angulo = limitarValor(maxAngulo - (maxAngulo - minAngulo) * posicaoRelativaAtualServos[servoIndex], limitesAngulo[servoIndex])
    setAngulo(servoIndex, angulo)
    return angulo

def posicaoRelativaParaAngulo(posicaoRelativa, servoIndex, image) :
    global limitesAngulo
    minAngulo = limitesAngulo[servoIndex][0]
    maxAngulo = limitesAngulo[servoIndex][1]
    amplitudeServo = maxAngulo - minAngulo
    angulo = maxAngulo - posicaoRelativa * amplitudeServo
    return angulo

def anguloParaPosicaoRelativa(angulo, servoIndex) :
    global limitesAngulo
    minAngulo = limitesAngulo[servoIndex][0]
    maxAngulo = limitesAngulo[servoIndex][1]
    amplitudeServo = limitesAngulo[servoIndex][1] - limitesAngulo[servoIndex][0]
    posicaoRelativa = (- angulo + maxAngulo) / amplitudeServo
    return posicaoRelativa

def modoLouco() :
    global limitesAngulo
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
    global limitesAngulo

    amplitudeServo0 = limitesAngulo[0][1] - limitesAngulo[0][0]
    amplitudeServo1 = limitesAngulo[1][1] - limitesAngulo[1][0]
    for i in range(40, -10, -1) :
        setAngulo(0, ((30-i) * (1/30) * amplitudeServo0 + limitesAngulo[0][0]))
        setAngulo(1, (i * (1/30) * amplitudeServo1 + limitesAngulo[1][0]))
        time.sleep(0.007)

def setModo(nomeModo) :
    global modos
    global modoAtual
    (nomeAtual, tempoAtual) = modoAtual
    if nomeAtual != nomeModo :
        modoAtual = (nomeModo, time.time())

def modoAtualEstaExpirado() :
    global modoAtual
    global modos
    valido = True
    (nomeModoAtual, tempoAtual) = modoAtual
    for (modo, tempoMin) in modos :
        if nomeModoAtual == modo :
            if time.time() - tempoAtual > tempoMin :
                valido = False
    return valido

def moverBaseadoNasFaces(faces, image) :
    global multiplicadorIntervalo
    global DEFAULT_MULTIPLICADOR_INTERVALO
    facesParaIgnorar = []
    areaFace = lambda face : face[2]*face[3]
    facesDecrescente = np.asarray(sorted(faces, key=areaFace, reverse=True))

    numeroFaces = facesDecrescente.shape[0]
    (x,y,w,h) = facesDecrescente[0]
    if x == 0 :
        numeroFaces = 0

    anguloServos = [0, 0]


    if numeroFaces == 0 :
        if time.time() - tempoUltimaDeteccao > 5 :
            multiplicadorIntervalo = DEFAULT_MULTIPLICADOR_INTERVALO
            (x, y, w, h) = (image.shape[1]/2, 0, 2, 0)
            anguloServos[0] = seguirFace(0, (x, y, w, h), image)
            anguloServos[1] = seguirFace(1, (x, y, w, h), image)
        elif time.time() - tempoUltimaDeteccao > 60 :
            if randint(0, 1000 * multiplicadorIntervalo) < 20 :
                if not somFoiTocadoRecente('semmovimento', 240) :
                    tocarSom('audio/semmovimento1.mp3')
        else :
            if randint(0, 10000 * multiplicadorIntervalo) < 4 :
                print('louco')
                modoLouco()
            else :
                anguloServos[0] = anguloAtualServos[0]
                anguloServos[1] = anguloAtualServos[1]
    else :
        multiplicadorIntervalo = int(0.8 * DEFAULT_MULTIPLICADOR_INTERVALO)
        if not somDoTipoFoiTocadoRecente('ola', 30) and not somDoTipoFoiTocadoRecente('salve', 30) :
            somNum = randint(1,5)
            if somNum <= 3:
                tocarSom('audio/ola{0}.mp3'.format(somNum))
            else :
                tocarSom('audio/salve{0}.mp3'.format(somNum - 3))
        if randint(0, 1000 * multiplicadorIntervalo) < 5 :
            if not somDoTipoFoiTocadoRecente('cantada', multiplicadorIntervalo*120) :
                somNum = randint(1, 5)
                #tocarSom('audio/cantada{0}.mp3'.format(somNum))
        if numeroFaces == 1 :
            anguloServos[0] = seguirFace(0, facesDecrescente[0], image)
            anguloServos[1] = seguirFace(1, facesDecrescente[0], image)
        elif numeroFaces == 2 :
            xFace = lambda face : face[0]
            facesDirParaEsq = np.asarray(sorted(faces, key=xFace, reverse=True))

            segundaFaceValida = lambda faces: faces[1][0] != 0
            if (segundaFaceValida(facesDirParaEsq)) :
                anguloServos[0] = seguirFace(0, facesDirParaEsq[1], image)
            else :
                anguloServos[0] = seguirFace(0, facesDirParaEsq[0], image)
            anguloServos[1] = seguirFace(1, facesDirParaEsq[0], image)
        else :
            #muita gente
            anguloServos[0] = seguirFace(0, facesDecrescente[0], image)
            anguloServos[1] = seguirFace(1, facesDecrescente[0], image)
            if randint(0, 100 * multiplicadorIntervalo) < 75 :
                if not somDoTipoFoiTocadoRecente('muitaspessoas') :
                    #75% de chance de dizer que está nervosa
                    somNum = randint(1,3)
                    tocarSom('audio/muitaspessoas{0}.mp3'.format(somNum))
                    if randint(0, 100 * multiplicadorIntervalo) < 10 :
                        modoEsconderOlhos()
    global args
    aleatorio = randint(0, 2000 * multiplicadorIntervalo)
    if aleatorio < 40 :
        if aleatorio < 15 :
            aleatorio2 = randint(0, 10)
            if aleatorio2 < 5 :
                somNum = randint(2,11)
                if (somNum <= 7):
                    tocarSom('audio/aleatorio{0}.mp3'.format(somNum))
                elif args.processo:
                    tocarSom('audio/processo{0}.mp3'.format(somNum-7))
            elif not args.no_geek :
                somNum = randint(1,8)
                tocarSom('audio/AleatorioGeek{0}.mp3'.format(somNum))
        elif aleatorio < 20 :
            tocarSom('audio/aleatorio1.mp3')
        elif aleatorio < 26 :
            if not somDoTipoFoiTocadoRecente('passa', 60 * multiplicadorIntervalo) :
                somNum = randint(1,3)
                tocarSom('audio/passa{0}.mp3'.format(somNum))
        else :
            if not somDoTipoFoiTocadoRecente('foto', 60 * multiplicadorIntervalo) :
                somNum = randint(1,5)
                tocarSom('audio/foto{0}.mp3'.format(somNum))

    return anguloServos

def movimentoInicial() :
    global limitesAngulo
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

cv2.setNumThreads(4)
pygame.init()
pygame.mixer.init()
initArduino()
globals()

somInicial()
movimentoInicial()

start = time.time()
i = 0



while pygame.mixer.music.get_busy() :
    pass

while time.time() - start < limiteTempo and capturaVideo.isOpened():
    if (not args.no_facebook) :
        if (time.time() - tempoUltimosDadosFacebook > 20) :
            requestDadosFacebook(PAGE_ID, ACCESS_TOKEN)
            print('likes: {0}'.format(likesFacebook))
    ret, frame = capturaVideo.read()
    cv2.resize(frame, (400,300))
    temFacesNaImagem = True
    try :
        faces = detectarFaces(frame, face_cascade)
    except LookupError :
        faces = np.array([(0,0,0,0)])
        temFacesNaImagem = False

    angulosServos = moverBaseadoNasFaces(faces, frame)
    if i % 1 == 0 :
        desenharResultado(frame, faces, angulosServos)

    i = i + 1
    current = time.time()
    print('Processed {0} frames in {1:.2f}s. Avg {2:.2f}fps\r'.format(i, (current-start), i/(current-start), end='\r'))
    
capturaVideo.release()
end = time.time()
print('Elapsed time: ' + str(end-start) + 's')
