
#!/usr/bin/python3
import time
import random
import RPi.GPIO as GPIO
import cv2
import numpy as np
import json
from picamera2 import MappedArray, Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
# python 3.6
import ftplib
import io
from paho.mqtt import client as mqtt_client

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import busio
import board
import adafruit_mlx90640

##############################
### MQTT #############
##########################

broker = '192.168.0.50'
port = 1883
topic_pub = "Kitchen_1/Vision_2/Out"
topic_sub = "Kitchen_1/Vision_2/In"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'
# username = 'emqx'
# password = 'public'

###################
##FTP##############
###################
ftp = ftplib.FTP()
ftp.connect('192.168.0.50', 14148)
ftp.login(user='jota', passwd='******')


gravar = False
lugar = 0
error = 0
motor = 0
temp_minimo = 0
temp_maximo = 0

####################
##ligar modo teste##
teste = False
####################

########################
#### Functions MQTT ####
########################
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    client.subscribe(topic_sub)
    client.username_pw_set("admin", "********")
    return client

def publish(client, texto):
    #msg_count = 0
    #while True:
    #    time.sleep(1)
	#msg = f"messages: {msg_count}"
	result = client.publish(topic_pub, texto)
	#result[0, 1]
	status = result[0]
	if status == 0:
		print(f"Send `{texto}` to topic `{topic_pub}`")
	else:
		print(f"Failed to send message to topic {topic_pub}")
	#msg_count += 1

# Definir a função de retorno de chamada
def on_message(client, userdata, message):
    global gravar
    global lugar
    global error
    global motor

    print("Mensagem recebida no tópico:", message.topic)
    #print("Conteúdo da mensagem:", str(message.payload.decode()))
    
    #formato dicionario = {"lugar": 1, "Id_number": 124, "Pre_process_type": 3}
    mensagem_encoded = str(message.payload.decode())
    
    # converter a string em um dicionário python
    mensagem_dict = json.loads(mensagem_encoded)

    # acessar os valores correspondentes
    mensagem_lugar = mensagem_dict["lugar"]
    mensagem_motor = mensagem_dict['motor']
    mensagem_error = mensagem_dict['error']
    mensagem_take_image = mensagem_dict["take_image"]

    print("lugar: " + str(mensagem_lugar))
    print("id_number: " + str(mensagem_motor))  
    print("mensagem_pre_process_type: " + str(mensagem_error))
    print("take image: " + str(mensagem_take_image))
    
    lugar = mensagem_lugar
    motor = mensagem_motor
    error = mensagem_error

    if mensagem_take_image == "yes_image" and error == "no_error":
        
        time.sleep(1)
        print("mudei status do gravar")
        gravar = True

################
## Video #######
################
# This is like opencv_face_detect_2.py, only we draw the face boxes on a
# recorded video. We have to use the "post_callback" to draw the faces because
# we want capture_buffer to get the image without face boxes, but then we
# want the boxes drawn before handing the image to the encoder.

colour = (0, 255, 0)
origin = (0, 30)
origin_machine = (50,30)
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 1
thickness = 2

parametro1 = 120
parametro2 = 170
minradio = 0
maxradio = 0


def take_image(client, image):
    global gravar
    global lugar
    global temp_maximo
    global temp_minimo

    #Capture image
    #image = picam2.capture()
    timestamp = time.strftime("%Y_%m_%d_%H_%M_%S")
    # Save image
    file_name = f"image_{timestamp}"
    
    file_dir = f"../images/{file_name}.jpg"
    file_dir_thermal = f"../images/{file_name}_thermal.png"
    time.sleep(1)
    cv2.imwrite(file_dir, image)
    print("nome_imagem: "+ file_name)
    print("nome_imagem: "+ file_dir_thermal)

    #########################
    ##Thermal Camera#######
    #######################
    # # Inicialize o barramento I2C
    i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)

    # Inicialize a câmera térmica
    mlx = adafruit_mlx90640.MLX90640(i2c, address=0x33)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ

    # Capture uma imagem
    frame = [0] * 768
    mlx.getFrame(frame)

    # Plote a imagem
    fig, ax = plt.subplots()
    im = ax.imshow(np.reshape(frame, (24, 32)), cmap='hot', vmin=20, vmax=40)
    fig.colorbar(im)

    # Encontre o valor mínimo e máximo
    temp_minimo = round(min(frame),0)
    temp_maximo = round(max(frame),0)

    print("Valor mínimo:", temp_minimo)
    print("Valor máximo:", temp_maximo)

    # Salvar a figura em um arquivo de imagem
    fig.savefig(file_dir_thermal)

    #########################
    #########################
    #########################

    process_image(file_name, client, lugar)



def process_image(file_name, client, lugar):
    global gravar
    global error
    global motor
    
    img = cv2.imread(f'../images/{file_name}.jpg')

    # Fazer uma cópia da imagem usando numpy
    img_copy = np.copy(img)

    # Converter a imagem para tons de cinza
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro Gaussiano para reduzir ruído

    img_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detectar círculos usando a transformada de Hough
    circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1.5, 20,
                            param1=parametro1, param2=parametro2, minRadius=minradio, maxRadius=maxradio)

    # Criar máscara circular com o mesmo tamanho do círculo detectado
    mask = np.zeros_like(gray)

    # Desenhar círculos detectados na imagem original
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r+10, (0, 255, 0), 2)
            cv2.circle(img, (x, y), 2, (0, 0, 255), 3)
            cv2.circle(mask, (x, y), r+10, 255, -1)

    # Obter as coordenadas delimitadoras da máscara

    y, x = np.where(mask == 255)
    if y.size > 0 and x.size > 0:
        top, left = np.min(y), np.min(x)
        bottom, right = np.max(y), np.max(x)
        h, w = bottom - top + 1, right - left + 1
        # print("x"+str(top))
        # print("y"+str(left))
        # print("w"+str(w))
        # print("h"+str(h))

        # Aplicar a máscara na imagem original para extrair a região de interesse
        result = cv2.bitwise_and(img, img, mask=mask)

        # Separa os canais de cor
        b, g, r = cv2.split(result)

        # Aplica a equalização adaptativa de contraste em cada canal de cor
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        b_cl = clahe.apply(b)
        g_cl = clahe.apply(g)
        r_cl = clahe.apply(r)

        # Combina os canais de volta para formar a imagem final
        cl_img = cv2.merge((b_cl, g_cl, r_cl))

        roi = cl_img[top:top+h, left:left+w]

        # cv2.imshow("Image", result)
        # cv2.imshow("Image_original", img)
        # cv2.imshow('Imagem Processada', roi)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        file_dir = f"../images/{file_name}_processed.jpg"
        cv2.imwrite(file_dir, roi)
        
        ftp_save(client, file_name)

    elif teste:

        print("modo teste")
        file_dir = f"../images/{file_name}_processed.jpg"
        cv2.imwrite(file_dir, img)

        # ftp_save(client, file_name)

    else:
        
        print("Não foram encontrados pixels brancos na máscara.")
        texto_a_por = {"lugar": lugar, 
                   "image": file_name,
                   "error": 1,
                   "motor": motor
                   }
    
        json_data = json.dumps(texto_a_por)
        publish(client,json_data)
        
        gravar = False

def ftp_save(client, file_name):
    global gravar
    global lugar
    global temp_maximo
    global temp_minimo

    file_dir = f"../images/{file_name}_processed.jpg"
    file_dir_thermal = f"../images/{file_name}_thermal.png"
    # Connect to FTP server and send file
    with open(file_dir, 'rb') as f:
        ftp.cwd("/images")
        print(ftp.pwd())
        ftp.storbinary(f'STOR {file_name}.jpg', f)

    with open(file_dir_thermal, 'rb') as f:
        ftp.cwd("/images")
        print(ftp.pwd())
        ftp.storbinary(f'STOR {file_name}_thermal.png', f)

    gravar = False
    texto_a_por = {"lugar": lugar, 
                   "image": file_name,
                   "error": 0,
                   "motor": motor,
                   "max_temp" : temp_maximo,
                   "min_temp" : temp_minimo
                    }
    
    json_data = json.dumps(texto_a_por)
    publish(client,json_data)

    gravar = False

#Function to show preview window
def show_preview(image, texto_a_por):
    ##cv2.putText(image, texto_a_por, origin, font, scale, colour, thickness)
    # Converter a imagem para tons de cinza
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Aplicar filtro Gaussiano para reduzir ruído
    img_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detectar círculos usando a transformada de Hough
    circles = cv2.HoughCircles(img_blur, cv2.HOUGH_GRADIENT, 1.5, 50,
                            param1=parametro1, param2=parametro2, minRadius=minradio, maxRadius=maxradio)

    # Desenhar círculos detectados na imagem original
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(image, (x, y), r+10, (0, 255, 0), 2)
            cv2.circle(image, (x, y), 2, (0, 0, 255), 3)

    # Show the image in a window
    cv2.imshow("Preview image", image)


def main():
    client = connect_mqtt()
    client.loop_start()
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (640, 480), "format":"RGB888"}))
    #config = picam2.create_preview_configuration()
    picam2.start()
    texto_preview = "Projeto Cook4me iniciado....."

    # Definir a função de retorno de chamada para o recebimento de mensagens
    client.on_message = on_message

    # # Iniciar o loop de recebimento de mensagens
    # client.loop_forever()
    
    while True:
        image = None
        image = picam2.capture_array()
        image_ai = image.copy()
        image_original = image.copy()

        #print("Status do gravar: {}".format(gravar))

        if cv2.waitKey(1) & 0xFF == ord('a'):


            take_image(client, image_original)  
        if gravar:
            take_image(client, image_original) 

        if image is not None:
            #show preview window
            #print(texto_a_por)
            show_preview(image_original, texto_preview)
            #do the machine_learning_process 
            #print("texto loop: "+ texto_a_por)

        # Verifica se o usuário pressionou a tecla 'q' para sair
        if cv2.waitKey(1) & 0xFF == ord('p'):
            print("pressionei o P para MQTT")
            publish(client,texto_preview)
    print("texto apos update: "+ texto_preview)
    print("terminei")    
    time.sleep(1)
    cv2.destroyAllWindows()
    print("terminou")

if __name__ == "__main__":
    main()
    







