#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <TaskScheduler.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <IRremote.hpp>
#include "pitches.h"

#include <WiFi.h>
#include <PubSubClient.h>

// Configuración de red WiFi
const char* ssid = "tetorras"; // Cambia por tu SSID de WiFi
const char* password = "pruebamqtt"; // Cambia por tu clave de WiFi  
  
// Configuración de MQTT
const char* mqtt_server = "192.168.68.134"; // Cambia por la dirección de tu servidor MQTT
const int mqtt_port = 1883; // Puerto MQTT (por defecto es 1883)
const char* mqtt_user = "ubicua";
const char* mqtt_password = "ubicua";
// Objeto de cliente WiFi
WiFiClient espClient;
PubSubClient client(espClient);

//Define para el giroscopio
Adafruit_MPU6050 mpu;

// Define para el sensor de ultrasonido
#define TRIGGER_PIN1 4  // Pin de salida del ultrasonido (conectar al pin TRIG del sensor)
#define ECHO_PIN1 2     // Pin de entrada del ultrasonido (conectar al pin ECHO del sensor)
#define MAX_DISTANCE 500  // Distancia máxima de medición en centímetros Ultrasonido
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // Crear un objeto NewPing Ultrasonido

// Define para el sensor de GPS
#define RXD2 16
#define TXD2 17
#define BUZZER_PIN 23

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Define para el sensor de velocidad
int encoder_pin = 32;
unsigned int rpm;
volatile byte pulses;
unsigned long timeold;
unsigned int pulsesperturn = 20;
const int wheel_diameter = 2;
float velocity = 0;

//Define para el sensor de infrarrojos
int IR = 34;     
bool estado = false; //False Bloqueado True Encendido

//Define para el zumbador
int melody[] = {
  NOTE_E6, NOTE_E7
};
int durations[] = {
  4, 4  // Duraciones en términos de la fracción de un segundo (1/4 significa una corchea)
};
 
  
  
void readUltrasonido();
void readGPS();
void readVelocimetro();
void counter(){
  pulses++;
}
void readInfrarrojos();
void zumbar();
//Creamos el scheduler
Scheduler scheduler;
//Creamos los threads
Task task_ultrasonido(500, TASK_FOREVER, &readUltrasonido, &scheduler, true);
Task task_gps(2000, TASK_FOREVER, &readGPS, &scheduler, true);
//Task task_velocimetro(50, TASK_FOREVER, &readVelocimetro, &scheduler, true);
Task task_infrarrojos(100, TASK_FOREVER, &readInfrarrojos, &scheduler, true);
Task task_mpu(1000, TASK_FOREVER, &readMPU, &scheduler, true);

void zumbar(){
  int size = sizeof(durations) / sizeof(int);
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
};



void initWiFi() {
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Conectando al servidor MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conexión MQTT exitosa");
      client.subscribe("sensores/respuesta"); // Suscribe a un tema si es necesario
    } else {
      Serial.println("Error en la conexión MQTT. Reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

void initMQTTServer(){
  client.setServer(mqtt_server, mqtt_port);
  reconnect();
  
}

void setup() {

  Serial.begin(115200);
  while (!Serial)
    delay(10); // Esperar a que se inicie el monitor serie
    
  // Setup del wifi
  initWiFi();
  initMQTTServer();
  // Asegúrate de que el cliente MQTT está conectado
  while (!client.connected()) {
    Serial.println("No conectado al broker MQTT");
  }


  Serial.println("Inicializando el MPU6050...");

  if (!mpu.begin()) {
    Serial.println("No se pudo encontrar un MPU6050. Verifique las conexiones y reinicie.");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 encontrado!");

  // Configurar el rango de aceleración y giroscopio
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //Setup Infrarrojos
  IrReceiver.begin(IR, DISABLE_LED_FEEDBACK);


  // Setup para el sensor GPS
  GPSSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //Setup para el sensor de velocidad
  pinMode(encoder_pin, INPUT);
  pulses = 0;
  rpm = 0;
  timeold = millis();
  attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, RISING);

  Serial.println("EMPEZAMOS");
  delay(2000);
  // Iniciar el planificador
  scheduler.startNow();
}

void loop() {
  scheduler.execute();
  
}
  

//Thread para ver si hay botella
void readUltrasonido() {
  if (estado != false){
  unsigned int distanciaObjeto = sonar1.ping_cm();
  Serial.print(distanciaObjeto);
  Serial.println(" cm");

  // Convertir el entero a String para poder publicarlo
  char buffer[10]; // Asumiendo que la distancia nunca superará 9999 cm
  snprintf(buffer, sizeof(buffer), "%u", distanciaObjeto);

  // Publicar la distancia convertida en string
  client.publish("sensores/distancia", buffer);
  }
}


//Thread para ver los datos del GPS
void readGPS() {
  gps.encode(GPSSerial.read());
  Serial.print("Lat: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Lng: ");
  Serial.println(gps.location.lng(), 6);
  String lat = String(gps.location.lat(), 6);
  String lng = String(gps.location.lng(), 6);
  String payload = "lat: " + lat + " lng: " + lng;
  client.publish("sensores/localizacion", payload.c_str());
}

//Thread para ver los datos del velocímetro
void readVelocimetro() {
  if (millis() - timeold >= 1000){
    detachInterrupt(digitalPinToInterrupt(encoder_pin));
    rpm = (pulses * 60) / ((millis() - timeold) / 1000 * pulsesperturn);
    velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000;
    Serial.print("Velocity: ");
    Serial.println(velocity);
    String v = String(velocity, 6);
    client.publish("sensores/velocidad", v.c_str());
    timeold = millis();
    pulses = 0;
    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, RISING);
  }
}

//Thread para ver los datos del infrarojos
void readInfrarrojos() { 
    if (estado != false){

      if (IrReceiver.decode()) {          
        Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);  
        // Verifica si el valor recibido es igual al valor hexadecimal deseado
        if (IrReceiver.decodedIRData.decodedRawData == 0xE31CFF00) { //Boton ok del mando
          // Cambia el valor de 'estado' de true a false o de false a true
          estado = !estado;
          // Puedes imprimir el nuevo estado para verificar
          Serial.print("Estado cambiado a: ");
          Serial.println(estado ? "true" : "false");
          client.publish("sensores/estado", estado ? "true" : "false");
        }

        zumbar(); // Función que activa un zumbador o similar
        IrReceiver.resume(); // Prepara para recibir el próximo dato            
    }           
  }
}

void readMPU() {
  if (estado != false){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calcular los ángulos de Euler */
  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180/M_PI;
  float pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180/M_PI;
  float yaw   = atan2(a.acceleration.z, a.acceleration.x) * 180/M_PI;  // Esto es una aproximación

  /* Imprimir los valores */
  Serial.print("Aceleración: ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Giroscopio: ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Ángulos de Euler: ");
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("°, Pitch: ");
  Serial.print(pitch);
  Serial.print("°, Yaw: ");
  Serial.print(yaw);
  Serial.println("°");

  // Crear el payload como una cadena
  String payload = "Roll(x): " + String(roll, 2) + ", Pitch(y): " + String(pitch, 2) + ", Yaw(z): " + String(yaw, 2);

  // Publicar los ángulos de Euler
  client.publish("sensores/angulos_euler", payload.c_str());
  }
}




