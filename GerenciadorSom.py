import pygame
import time


class GerenciadorSom:
    sonsRecentes: [str] = []
    tempoAntesDeRepetirSons: int = 0
    defaultTempoMaximo: int = 120

    def __init__(self, tempoAntesDeRepetirSons: int):
        pygame.init()
        pygame.mixer.init()

    def tocarSom(self, nomeArquivo: str):
        if not pygame.mixer.music.get_busy() and not self.somFoiTocadoRecente(nomeArquivo, self.tempoAntesDeRepetirSons):
            pygame.mixer.music.load(nomeArquivo)
            pygame.mixer.music.play()
            self.sonsRecentes.append((nomeArquivo, time.time()))
            print('Tocando: {0}'.format(nomeArquivo))

    def somFoiTocadoRecente(self, nomeArquivo: str, tempoMaximo: int = defaultTempoMaximo):
        foiTocado = False
        for (som, tempo) in self.sonsRecentes:
            if tempo > (time.time() - tempoMaximo):
                if som == nomeArquivo:
                    foiTocado = True
                    break
        return foiTocado

    def somDoTipoFoiTocadoRecente(self, prefixoSom: str, tempoMaximo: int = defaultTempoMaximo):
        foiTocado = False
        for i in range(1, 10):
            if self.somFoiTocadoRecente('audio/{0}{1}.mp3'.format(prefixoSom, i), tempoMaximo):
                foiTocado = True
                break
        return foiTocado

    def removerSonsAntigos(self):
        for (som, tempo) in self.sonsRecentes:
            if time.time() - tempo > self.defaultTempoMaximo * 5:
                self.sonsRecentes.remove((som, tempo))

    def somTocandoAgora(self):
        return pygame.mixer.music.get_busy()
