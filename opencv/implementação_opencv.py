import numpy as np
import cv2 as cv

#detecção de humanos na imagem
rosto_frente = cv.CascadeClassifier('c:/Users/lucas/Documents/opencv/haarcascade_frontalface_default.xml')
rosto_lado = cv.CascadeClassifier('c:/Users/lucas/Documents/opencv/haarcascade_profileface.xml')

cap = cv.VideoCapture(0) #captura da imagem da webcam quando o parâmetro é igual a 0
if not cap.isOpened():
    print("Erro de câmera")
    exit()

#matriz para engrossamento das bordas
k = np.ones((3,3), np.uint8)

while True:
    ret,video = cap.read()
    if not ret: 
        print("Erro de captura") 
        break

    video = cv.resize(video, (320, 240))
    
    #transformação para escala de cinza
    escala_cinza = cv.cvtColor(video, cv.COLOR_BGR2GRAY) 
    escala_cinza_invertido = cv.flip(escala_cinza, 1)

    #transformação para bordas
    bordas = cv.Canny(escala_cinza,100,300) 

    #bordas engrossadas
    bordas_en = cv.dilate(bordas, k, iterations=0)

    #transforma a imagem de bordas com 1 canal em uma imagem em 3 canais
    bordas_colorido = cv.cvtColor(bordas_en, cv.COLOR_GRAY2BGR) 

    #superposção das imagens
    superposicao = cv.addWeighted(video, 0.9, bordas_colorido, 0.5, 0)

    #detecção de humanos no video
    deteccao_frontal = rosto_frente.detectMultiScale(escala_cinza, 1.1, 4)
    detecçao_lateral_esquerda = rosto_lado.detectMultiScale(escala_cinza, 1.1, 4)
    detecçao_lateral_direita = rosto_lado.detectMultiScale(escala_cinza_invertido, 1.1, 4)

    for (x, y, w, h) in deteccao_frontal: 
        cv.rectangle(superposicao, (x, y), (x+w, y+h), (255, 0, 0), 2)

    for (x, y, w, h) in detecçao_lateral_esquerda: 
        cv.rectangle(superposicao, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    for (x, y, w, h) in detecçao_lateral_direita: 
        x = escala_cinza.shape[1] - x - w 
        cv.rectangle(superposicao, (x, y), (x + w, y + h), (0, 0, 255), 2)

    #calculo do centro do video
    video_centro = video.shape[1] // 2

    #calculo do centro do rosto em diferentes angulos
    if len(deteccao_frontal) > 0: 
        (x, y, w, h) = deteccao_frontal[0]
    elif len(detecçao_lateral_esquerda) > 0: 
        (x, y, w, h) = detecçao_lateral_esquerda[0]
    elif len(detecçao_lateral_direita) > 0: 
        (x, y, w, h) = detecçao_lateral_direita[0] 
        x = escala_cinza.shape[1] - x - w
    else: x, y, w, h = 0, 0, 0, 0

    if w > 0: 
        face_center_x = x + w // 2 

        #essa é a saída, em pixels, do deslocamento necessário do motor
        move_x = video_centro - face_center_x 
        cv.putText(superposicao, f"Move X: {move_x}px", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)  
    

    #video dos contornos
    cv.imshow('Video Bordas',bordas)

    #video da soma do normal com os contornos
    cv.imshow('Video Normal + Bordas',superposicao)

    if cv.waitKey(1) & 0xFF == ord('q'): #fecha o loop pressionando a tecla q
        break
cap.release()
cv.destroyAllWindows()