/*-------------------------------------------------------------------------
 * 
 * 
 * CONEXIONES
 * SENSOR TEMPERATURA HUMEDAD - PIN D3 - GPIO0
 * SERVO                      - PIN D2 - GPIO4
 * SENSOR PIR                 - PIN D1 - GPIO5
 * RELE                       - PIN D5 - GPIO14
 * 
 * 
 * 
 */

#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h" // including the library of DHT11
#define DHTTYPE DHT11 // DHT 11 sensor usado
#define dht_dpin 0 //GPIO0 PIN D3
#define rele 14 //GPIO14 PIN D5
#define pirSensor 5 //GPIO5 PIN D1

Servo servo;
DHT dht(dht_dpin, DHTTYPE);
// DEFINICIONES---------- DATOS RED WIFI --------------------------
const char* ssid = "xxxx";
const char* password = "xxxxx";
float Setpoint=20.0;
// DEFINICIONES------------- MQTT ---------------------------------
const char* mqtt_server = "192.168.3.25";
WiFiClient espClient;
PubSubClient client(espClient);
//DEFINICIONES------------- VARIABLES GLOBALES--------------------
long lastMsg = 0;
unsigned long lastPir = 0; // Tempo en que se activo el PIR por ultima vez
char msg[150];
int value = 0;
boolean releState = LOW , ilAutomatica = LOW;
//***********************************************************************************
//********************************************SETUP**********************************

void setup(void){
//----------------- Declaracion de pines --------------------------------------
  pinMode(rele, OUTPUT);
  pinMode(pirSensor, INPUT_PULLUP);
  servo.attach(4); //Define donde esta conecatado el servo  
  attachInterrupt(digitalPinToInterrupt(pirSensor),detectarMovimiento, RISING); // Set Interrup PIR
  dht.begin();                        //Inicializa sensor de temperatura y humedad
  Serial.begin(115200);               //Inicializa el puerto serie
  setup_wifi();                       //Llama a la subrutina para inicializar la conexion WiFi
  client.setServer(mqtt_server, 1883);//Declara el servidor MQTT  
  client.setCallback(callback);
  Serial.println("****************************************");
  Serial.println("Humedad y temperatura con DHT11");
  Serial.println("****************************************");
  delay(4000);//tiempo para mostrar el mensaje anterior
}

//***********************************************************************************
//***************************************SETUP WIFI**********************************
//Esta funcion realiza la conexion a la red WIFI

void setup_wifi() {
  delay(10);
  // Conexion a la red WiFi
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("Conectado a WiFi");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//***********************************************************************************
//*************************************RUTINA INTERRUPCION PIR***********************
ICACHE_RAM_ATTR void detectarMovimiento() {
  Serial.println("Movimiento detectado");
  releState=digitalRead(rele);
  if(!releState && ilAutomatica){
    digitalWrite(rele, HIGH);
    lastPir = millis();
  }
}
  
//***********************************************************************************
//************************* RECEPCION DE MENSAJES MQTT ******************************
void callback(String topic, byte* payload, unsigned int length) {
  String messageTemp;
  Serial.print("Mensaje MQTT entrante [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.print(messageTemp);
  
  Serial.println();

   if(topic =="/frch/a2/spt"){
         Serial.print("Nueva Set point de temperatura ");
         Setpoint=messageTemp.toFloat();
         Serial.print(Setpoint);
   }
   else if(topic =="/frch/a2/laut"){
      if(messageTemp=="true"){
            Serial.print("RM - Iluminacion en automatico");
            ilAutomatica=HIGH;
            digitalWrite(rele,LOW);
      }
      else if(messageTemp=="false"){
            Serial.print("RM - Iluminacion en manual");
            ilAutomatica=LOW;
      }
   }
  
   else Serial.print("error en topic");
  Serial.println();
}
//********************************* Funcion para reconectar el cliente al servidor MQTT **********************
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado al Broker");
      // Once connected, publish an announcement...
      client.publish("/frch/a2", "Enviando el primer mensaje");
      // ... and resubscribe
      client.subscribe("/frch/a2/spt");
      client.subscribe("/frch/a2/laut");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//************************************************************************************************************** 
//***************************************Funcion principal LOOP************************************************* 

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int cierre=(90*(t-(Setpoint+0.5)))+180; // calcula el grado de apertura intermedio
  int apservo=0; 
  long unsigned int tiempoActual;
  char msgTemp[256],temp[5];
  //chequeo de tiempo 
  tiempoActual = millis();
  releState=digitalRead(rele);
/**************Control de apertura*********/
  
  if (t>(Setpoint+0.5)) {
          servo.write(180); // abre
          delay(1000); // tiempo entre lecturas
          }
  if (t<(Setpoint-0.5)) {
          servo.write(90); // abre
          delay(1000); // tiempo entre lecturas
          }
  if ((t>(Setpoint-0.5)) && (t<(Setpoint+0.5))){
          servo.write(cierre); // cierra en punto intermedio
          delay(1000);
          }
  apservo=180 - servo.read();
  Serial.print("Humedad = ");
  Serial.print(h);
  Serial.print(" % ");
  Serial.print("|Temperatura = ");
  Serial.print(t);
  Serial.print(" °C ");
  Serial.print("|Apertura=");
  Serial.print(apservo);
  Serial.print("°");
  Serial.print("|SetPoint=");
  Serial.print(Setpoint);
  Serial.print("|Ilum Auto=");
  Serial.println(releState);
  Serial.println("------------------------------------------");

  long now = millis();        //---Milisegundos actuales
/*******************MQTT******************/  
  if (!client.connected()) {  //Si se desconecto del brocker volver a conectar
    reconnect();
  }
  client.loop();
 
  if (now - lastMsg > 2000) {
    lastMsg = now;
    dtostrf(t,2,2,temp);
    snprintf (msg, 75, "{\"t\":%s,\"h\":%i,\"ap\":%i,\"sp\":%i}", temp,(int)h,apservo,(int)Setpoint);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("/frch/a2", msg);
  }
 if (!ilAutomatica && !releState){
      Serial.println("L - Encendido luces manual");
      digitalWrite(rele,HIGH);
 }
 else if (ilAutomatica && (now - lastPir > 120000)&& releState) {
      digitalWrite(rele,LOW);
      Serial.println("L - Se apagan las luces");
 }
}
