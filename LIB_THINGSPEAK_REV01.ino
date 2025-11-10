/*********************************************************************************************************************************************************************************
  AUTOR: WALTER DAWID RETZER
  
  PROJETO: MQTT - ENVIO DO VALOR DE TEMPERATURA AO THINGSPEAK UTILIZANDO ARDUINO OPTA FINDER
  VERSION: 1.0.1
  DATA: 05/11/2025
  ARQUIVO: LIB_THINGSPEAK_REV01

  COMENTÁRIOS COM AS ALTERAÇÕES PARA REGISTRO DE CONTROLE DAS VERSÕES:

  1.0.0:          INCLUÍDO LIB PUBSUBCLIENT PARA REALIZAR A CONEXÃO COM O THINGSPEAK(FASE DE TESTE);
  1.0.1:          INCLUÍDO ABA SECRETS.H PARA GESTÃO SEGURA DAS SENHAS;
                                                                
********************************************************************************************************************************************************************************/

//---Libs Utilizadas:
#include <PubSubClient.h>  //LIB para permitir comunicação via protocolo MQTT
#include <WiFi.h>          //LIB para permitir conexão WiFi
#include "Secrets.h"       //LIB para armazenamento seguro das senhas

//---Entrada Analógica(Opta)
#define AI0 A1  // canal 1 (entrada analogica A1 (0 a 10V))

//---WIFI
const char WIFI_SSID[] = SECRET_WIFI_SSID;
const char WIFI_PASS[] = SECRET_WIFI_PASS;

//---TIMERS
const unsigned long SEND_INTERVAL_MS = 30000UL;  // 30s
const unsigned long BLINK_INTERVAL_MS = 1000UL;  // 1s
unsigned long tSendPrev = 0;
unsigned long tBlinkLed = 0;

//---Variavíes Globais
bool ledState = true;
float temperature;

//---Credenciais para acesso ao Broker do ThingSpeak:
const long channelID = SECRET_MQTT_CHANNEL_ID;
const int mqttPort = SECRET_MQTT_PORT;
const char server[] = SECRET_MQTT_SERVER;
const char mqttUserName[] = SECRET_MQTT_USER;
const char clientID[] = SECRET_MQTT_ID;
const char mqttPass[] = SECRET_MQTT_PASS;

//---Clients
WiFiClient client;
PubSubClient mqttClient(server, mqttPort, client);


/*************************************/  //Funções///*************************************/

//---Função de fazer o Led piscar a cada 1s:
static void blinkOnce() {
  ledState = !ledState;
  digitalWrite(LEDG, ledState);
}


//---Função para publicar a mensagem ao ThingSpeak:
void mqttPublish(long pubChannelID, String message) {
  String topicString = "channels/" + String(pubChannelID) + "/publish";
  mqttClient.publish(topicString.c_str(), message.c_str());
}


//---Função para realizar a Conexão MQTT com o server ThingSpeak:
void mqttConnect() {
  digitalWrite(LEDB, HIGH);
  // Checa se o server esta conectado.
  if (!mqttClient.connected()) {
    // Conecta-se ao MQTT Broker usando as Credenciais do ThingSpeak:
    if (mqttClient.connect(clientID, mqttUserName, mqttPass)) {
      Serial.print("MQTT Conectado com sucesso ao Server: ");
      Serial.print(server);
      Serial.print(" na porta:");
      Serial.print(mqttPort);
      Serial.println(" do Broker ThingSpeak.");
    } else {
      Serial.print("Conexão MQTT em Falha! Status da Falha = ");
      Serial.print(mqttClient.state());
      Serial.println("Próxima tentativa de conexão em 30s...");
    }
  }
  digitalWrite(LEDB, LOW);
}


//---Tenta (re)conectar ao Wi-Fi de forma não-bloqueante:
void ensureWifiConnected() {

  if (WiFi.status() == WL_CONNECTED) return;  // já conectado, retorna

  Serial.print("Conectando WiFi em ");
  Serial.print(WIFI_SSID);
  Serial.println(" ...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi conectado com Sucesso à rede: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Intensidade do Sinal(RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}


//---Função de Configuração Inicial:
void setup() {
  Serial.begin(115200);
  delay(200);  // pequeno respiro para abrir Serial

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  delay(1000);

  pinMode(LEDB, OUTPUT);
  pinMode(LEDG, OUTPUT);

  // Primeira tentativa de Wi-Fi (rápida, sem bloquear)
  ensureWifiConnected();
  if (WiFi.status() == WL_CONNECTED) {
    //Exibindo parâmetros da Rede WiFi:
    Serial.print("WiFi conectado com Sucesso à rede: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Intensidade do Sinal(RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println("dBm");
  }

  digitalWrite(LEDB, HIGH);
  delay(5000);
  digitalWrite(LEDB, LOW);
}


//---Função que é repetida continuamente:
void loop() {
  unsigned long now = millis();

  // Verifica se o tempo de 30s foi atingido e conecta-se ao ThingSpeak:
  if (now - tSendPrev >= SEND_INTERVAL_MS) {
    tSendPrev = now;

    //Verifica se o Wi-Fi esta com conexão ativa:
    ensureWifiConnected();

    analogReadResolution(12);                  // Resolução de 12bits -> 4095
    int leitura = analogRead(A1);              // Valor Convergido de Analog pra Digi: 0–4095 (12 bits)
    temperature = (leitura * 100.0) / 4095.0;  // Converte para temperatura (10V = 4095 -> 100°C)

    Serial.println("Tempo de envio de 30s atingido. Preparando envio dos dados...");
    Serial.print("Input A1: ");
    Serial.println(leitura);
    Serial.print("Temperatura: ");
    Serial.print(temperature, 1);
    Serial.println("°C");

    // Conecta-se ao MQTT Broker do ThingSpeak para realizar a publicação da Temperatura:
    mqttConnect();
    mqttPublish(channelID, (String("field1=") + String(temperature, 1)));
    Serial.println("Valor da Temperatura enviada ao Broker ThingSpeak!");
  }

  // Pisca a cada 1s, só para indicar vida
  if (now - tBlinkLed >= BLINK_INTERVAL_MS) {
    tBlinkLed = now;
    blinkOnce();
  }
}
