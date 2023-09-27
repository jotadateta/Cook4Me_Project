#!/usr/bin/python3
import time
import random
# python 3.6
import ftplib
import io
from paho.mqtt import client as mqtt_client
import numpy as np
import json

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
    client.username_pw_set("admin", "*******")
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

    if mensagem_take_image == 1 and error == 0:
        print("mudei status do gravar")
        gravar = True


def main():
    client = connect_mqtt()
    client.loop_start()

    # Definir a função de retorno de chamada para o recebimento de mensagens
    client.on_message = on_message

if __name__ == "__main__":
    main()