import pygame
import time

def tocarSom(nomeArquivo) :
    pygame.mixer.music.load(nomeArquivo)
    pygame.mixer.music.play()
    #while pygame.mixer.music.get_busy(): 
    #    pygame.time.Clock().tick(10)

pygame.init()
print('iniciou')
tocarSom('audio/start_race.mp3')
print('terminou')
for i in range(100) :
    print(i)
    time.sleep(0.1)