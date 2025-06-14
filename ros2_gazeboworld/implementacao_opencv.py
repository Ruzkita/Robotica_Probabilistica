import numpy as np
import cv2 as cv


def main(cv_image):
    # Verifica se a imagem foi recebida corretamente
    if cv_image is None:
        print("Erro: Imagem não recebida corretamente")
        return

    # Matriz para engrossamento das bordas
    k = np.ones((3, 3), np.uint8)

    # Transformação para escala de cinza
    escala_cinza = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY) 
    escala_cinza_invertido = cv.flip(escala_cinza, 1)

    # Transformação para bordas
    bordas = cv.Canny(escala_cinza, 100, 300)

    # Bordas engrossadas
    bordas_en = cv.dilate(bordas, k, iterations=0)

    # Transforma a imagem de bordas com 1 canal em uma imagem em 3 canais
    bordas_colorido = cv.cvtColor(bordas_en, cv.COLOR_GRAY2BGR)

    # Superposição das imagens
    superposicao = cv.addWeighted(cv_image, 0.9, bordas_colorido, 0.5, 0)

    # Exibe o vídeo das bordas
    cv.imshow('Video Bordas', bordas)

    # Exibe o vídeo da soma do normal com os contornos
    cv.imshow('Video Normal + Bordas', superposicao)

    cv.waitKey(1)
