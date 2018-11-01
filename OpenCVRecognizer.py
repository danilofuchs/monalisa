import cv2
import numpy as np


class OpenCVRecognizer:
    cameraIndex: int = 0
    capturaVideo = 0
    faceCascade = 0
    resolucaoCaptura: (int, int) = (1280, 720)
    resolucaoDeteccao: (int, int) = (640, 360)
    converterCinza: bool = True

    def __init__(self, cameraIndex: int, resolucaoCaptura: (int, int), resolucaoDeteccao: (int, int), cascadeClassifierPath: str, converterCinza: bool):
        cv2.setNumThreads(4)
        self.cameraIndex = cameraIndex
        self.capturaVideo = cv2.VideoCapture(cameraIndex)
        self.resolucaoCaptura = resolucaoCaptura
        self.resolucaoDeteccao = resolucaoDeteccao
        self.capturaVideo.set(cv2.CAP_PROP_FRAME_WIDTH, resolucaoCaptura[0])
        self.capturaVideo.set(cv2.CAP_PROP_FRAME_HEIGHT, resolucaoCaptura[1])
        self.faceCascade = cv2.CascadeClassifier(cascadeClassifierPath)
        self.converterCinza = converterCinza

    def isOpened(self):
        return self.capturaVideo.isOpened()

    def snapshot(self):
        ret, frame = self.capturaVideo.read()
        return ret, frame

    def detectarFaces(self):
        ret, image = self.snapshot()
        frame = image
        cv2.resize(frame, self.resolucaoDeteccao)
        if (self.converterCinza):
            # Converte imagem para preto/branco (3x mais rapido)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Encontra faces pelo modelo treinado
        faces = self.faceCascade.detectMultiScale(frame, 1.1, 5)

        # Encontrou faces?
        if len(faces) > 0:
            # if not pygame.mixer.music.get_busy():
            #    tocarSom('audio/oi_gato.mp3')
            return faces, ret, image
        else:
            return np.array([(0,0,0,0)]), ret, image
            # raise LookupError('No face found')

    def release(self):
        self.capturaVideo.release()
