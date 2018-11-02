from typing import Callable, NoReturn
import grequests
import json
from random import randint
import time


class GerenciadorFacebook:
    apiEndpoint = "https://graph.facebook.com/v2.4/"
    pageId: str = ""
    accessToken: str = ""
    tempoUltimaVerificacao: int = 0
    likes: int = 0
    novoLike: bool = False
    checked: bool = True

    def __init__(self, pageId: str, accessToken: str):
        self.pageId = pageId
        self.accessToken = accessToken
        self.fbGraphUrl = self.apiEndpoint+self.pageId + \
            "?fields=fan_count&access_token="+self.accessToken

    def requestDadosFacebook(self):
        print("Mandando requisição")
        req = grequests.get(self.fbGraphUrl, hooks=dict(
            response=self.receberDadosFacebook))
        request = req.send()
        self.tempoUltimaVerificacao = time.time()

    def receberDadosFacebook(self, response, *args, **kwargs):
        print("Recebido")
        pageData = json.loads(response.content)
        newLikeCount = pageData['fan_count']
        print(newLikeCount)
        # print(pageData['error'])
        '''
        if (pageData['error'] != None):
            print("Erro de autenticação facebook")
            return
        '''
        if newLikeCount != self.likes:
            if newLikeCount > self.likes:
                self.novoLike = True
                self.checked = False
            else:
                self.novoLike = False
                self.checked = False
            self.likes = newLikeCount
            '''
            random = randint(0, 3)
            global start
            if time.time() - start > 20 :
                #Não toca na primeira iteração
                som.tocarSom('audio/like{0}.mp3'.format(random))
            '''

    def getLikeCount(self) -> (bool, int):
        self.checked = True
        print(self.novoLike)
        if self.novoLike:
            self.novoLike = False
            return True, self.likes
        else:
            return False, self.likes
