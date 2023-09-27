#include <AccelStepper.h>
#include <ArduinoJson.h>

// MQTT
#include <Ethernet.h>
#include <PubSubClient.h>

// Definir as conexões do motor com o CNC Shield
#define motor1Step 2
#define motor1Dir 3

#define motor2Step 7
#define motor2Dir 6

// Define a velocidade máxima do motor em RPM
#define MAX_SPEED 200

// Define a aceleração do motor em passos por segundo ao quadrado (1.8 m/s²)
#define ACCELERATION 2048

// Configurar o stepper motor com a biblioteca AccelStepper
AccelStepper stepper1(1, motor1Step, motor1Dir);
AccelStepper stepper2(1, motor2Step, motor2Dir);

// Definir o pino do switch de limite de curso
#define switch1 9

//////////////////////////////////////////////////////////////////////////////////////
// Define o endereço MAC e o endereço IP do seu shield Ethernet
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0xA9, 0xC9 };
IPAddress mqtt_server(192, 168, 0, 50);  // ip server

IPAdress avenida_server(192, 168, 0, 50);

IPAddress ip(192, 168, 0, 144);
IPAddress subnet(255, 255, 255, 0);  // máscara de sub-rede
IPAddress gateway(192, 168, 0, 100);  // gateway padrão
IPAddress dns(0, 0, 0, 0);  // endereço IP do servidor DNS


// Cria uma instância da biblioteca Ethernet e da biblioteca PubSubClient
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

/////////////////////////////////////////////////////////////////

int motor_1_home = false;

int pos_atual_motor1 = 0;
int pos_atual_motor2 = 0;
int pos_atual_motor3 = 0;

void setup() {
  Serial.begin(9600);

  ///////////////////////////////////////////////////////////////////////
  // Inicializa a comunicação serial
  Serial.begin(9600);
  
  // Inicializa o shield Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);

  // Define o servidor MQTT e a porta a serem usados
  mqttClient.setServer(mqtt_server, 1883);
  // Define a função de callback para lidar com mensagens recebidas
  mqttClient.setCallback(callback); 

  //////////////////////////////////////////////////////////////////////

  // Configurar o modo do pino do switch como entrada
  pinMode(switch1, INPUT_PULLUP);
  
  // Configurar o modo dos pinos do motor1 como saída
  pinMode(motor1Step, OUTPUT);
  pinMode(motor1Dir, OUTPUT);

  // Configurar o modo dos pinos do motor2 como saída
  pinMode(motor2Step, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  
  // Define a velocidade máxima do motor em passos por segundo
  stepper1.setMaxSpeed(MAX_SPEED * 200 / 60); // Converte RPM para passos por segundo
  // Define a aceleração do motor em passos por segundo ao quadrado
  stepper1.setAcceleration(ACCELERATION);

    // Define a velocidade máxima do motor em passos por segundo
  stepper2.setMaxSpeed(MAX_SPEED * 200 / 60); // Converte RPM para passos por segundo
  // Define a aceleração do motor em passos por segundo ao quadrado
  stepper2.setAcceleration(ACCELERATION);

  home_motor(1);
  home_motor(2);
}

void loop() {
  // Chamar a função home passando o número do motor como argumento
  /////////////////////////////////////////////////////////////////////
  // Verifica a conexão com o servidor MQTT
  if (!mqttClient.connected()) {
    reconnect();
  }
  // Aguarda a chegada de novas mensagens do servidor MQTT
  mqttClient.loop();
  delay(100);
  // Publica uma mensagem no tópico MQTT
  
  ////////////////////////////////////////////////////////////////////

  // home(1);
  // home(2);
  // delay(2000);
  // mover_motor(2, 2);
  // delay(2000);
  // mover_motor(1, 4);
  // Serial.println("Posição atual motor1 state1: " + String(stepper1.currentPosition() * 0.04) + " mm");
  // delay(2000);
  // mover_motor(1, 1);
  // Serial.println("Posição atual motor1 state2: " + String(stepper1.currentPosition() * 0.04) + " mm");
  //delay(100000);
  // Outras operações do programa
}

// Definir a função home
void home_motor(int motorNumber) {
  // Verificar qual motor deve ser movido
  switch (motorNumber) {
    case 1:
      // Mover o motor 1 em sentido inverso até atingir o switch de limite de curso
      while (digitalRead(switch1) == HIGH) {
        stepper1.setSpeed(MAX_SPEED * 200 / 60); // Converte RPM para passos por segundo
        stepper1.runSpeed();
      }
      // Definir a posição atual como 0
      stepper1.setCurrentPosition(0);
      delay(100);
      // Retornar o motor à posição inicial
      stepper1.moveTo(100);
      stepper1.runToPosition();
      motor_1_home = true;
      break;
      
    case 2:
      // Mover o motor 2 em sentido inverso até atingir o switch de limite de curso
      while (digitalRead(switch1) == HIGH) {
        stepper2.setSpeed(MAX_SPEED * 200 / 60); // Converte RPM para passos por segundo
        stepper2.runSpeed();
      }
      // Definir a posição atual como 0
      stepper2.setCurrentPosition(0);
      delay(100);
      // Retornar o motor à posição inicial
      stepper2.moveTo(100);
      stepper2.runToPosition();
      break;
      
  }
}

void ir_posicao(int motorNumber, long posicao) {
  int posicao_mover; // declarar a variável posicao_mover

  if (posicao == 1) {
    posicao_mover = 2000; // atualiza a posição atual do motor 
  } else if (posicao == 2) {
    posicao_mover = 4000; // atualiza a posição atual do motor 2
  } else {
    posicao_mover = 6000; // atualiza a posição atual do motor 3
  }

  switch (motorNumber) {
    case 1:
      // Mover o motor 1 para a posição especificada
      stepper1.moveTo(posicao_mover);
      while (stepper1.distanceToGo() != 0) { // verifica se o motor está se movendo
        // verifica se o motor atingiu o limite mecânico ou saiu do intervalo seguro
        if (digitalRead(switch1) == LOW) {
          // Se o limite de segurança foi violado, inverte o movimento do motor
          stepper1.moveTo(stepper1.currentPosition() - stepper1.distanceToGo());
          stepper1.setSpeed(0); // define a velocidade como zero para parar imediatamente
          break;
        }
        stepper1.run();
      }
    break;
    case 2:
      // Mover o motor 2 (eixo Y) para a posição especificada
      stepper2.moveTo(posicao_mover);
      while (stepper2.distanceToGo() != 0) { // verifica se o motor está se movendo
        // verifica se o motor atingiu o limite mecânico ou saiu do intervalo seguro
        if (digitalRead(switch1) == LOW) {
          // Se o limite de segurança foi violado, inverte o movimento do motor
          stepper2.moveTo(stepper2.currentPosition() - stepper2.distanceToGo());
          stepper2.setSpeed(0); // define a velocidade como zero para parar imediatamente
          break;
        }
        stepper2.run();
      }
      break;
  }
}

void mover_motor(int motor, int pos_desejada) {
  Serial.println("motor: "+ String(motor) + " posição: "+ String(pos_desejada));
  ir_posicao(motor, pos_desejada); // move o motor para a posição desejada
  if (motor == 1) {
    pos_atual_motor1 = pos_desejada; // atualiza a posição atual do motor 
  } else if (motor == 2) {
    pos_atual_motor2 = pos_desejada; // atualiza a posição atual do motor 2
  } else {
    pos_atual_motor3 = pos_desejada; // atualiza a posição atual do motor 3
  }
}

// Função de callback para lidar com mensagens recebidas
void callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Message arrived [");
  Serial.println(topic);
  

  // Cria um objeto JsonDocument com tamanho suficiente para armazenar a mensagem recebida
  StaticJsonDocument<200> doc;

  // Analisa a mensagem recebida e armazena no objeto JsonDocument
  DeserializationError error = deserializeJson(doc, payload, length);
  
  // Verifica se houve erro na análise da mensagem
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Obtém os valores dos itens e armazena em variáveis
  bool home = doc["home"];
  int motor = doc["motor"];
  int spot = doc["spot"];

  // Imprime os valores das variáveis na porta serial
  Serial.print("home: ");
  Serial.println(home);
  Serial.print("motor: ");
  Serial.println(motor);
  Serial.print("spot: ");
  Serial.println(spot);

  // Converte o payload em uma string
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Exibe a mensagem recebida no monitor serial
  //Serial.println(message);

  // Verifica o valor do item "home" e chama a função correspondente
  if (home == 1) {
    home_motor(motor);
  } else {
    mover_motor(motor, spot);
  }

}

// Reconecta-se ao servidor MQTT se a conexão for perdida
void reconnect() {
  // Tenta se reconectar
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Tenta se conectar
    if (mqttClient.connect("jota_da_tuga")) {
      Serial.println("connected");
      mqttClient.publish("jota/teste", "Hello tropitas da pesada!");
      // Inscreva-se no tópico MQTT
      mqttClient.subscribe("jota/teste");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");

      // Aguarda um tempo antes de tentar se reconectar novamente
      delay(5000);
    }
  }
}

