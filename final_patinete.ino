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
const char* ssid = "patinete";         // Cambia por tu SSID de WiFi
const char* password = "patinet3030";  // Cambia por tu clave de WiFi

// Configuración de MQTT
const String idPatinete = "1";
const char* mqtt_server = "homasromrod.duckdns.org";  // Cambia por la dirección de tu servidor MQTT
const int mqtt_port = 55000;                          // Puerto MQTT (por defecto es 1883)
const char* mqtt_user = "ubicua";
const char* mqtt_password = "ubicua";

String topic = "";
String localizacion = "";
String payload = "";
bool estado = true;  //False Bloqueado True Encendido
bool accidente = false;


// Objeto de cliente WiFi
WiFiClient espClient;
PubSubClient client(espClient);

//Define para el giroscopio
Adafruit_MPU6050 mpu;
float umbralAceleracion = -10;  // Umbral de desaceleración en m/s²
float umbralRotacion = 1.5;
bool estadoAc = 0;
// Define para el sensor de ultrasonido
#define TRIGGER_PIN1 2                                  // Pin de salida del ultrasonido (conectar al pin TRIG del sensor)
#define ECHO_PIN1 4                                     // Pin de entrada del ultrasonido (conectar al pin ECHO del sensor)
#define MAX_DISTANCE 500                                // Distancia máxima de medición en centímetros Ultrasonido
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // Crear un objeto NewPing Ultrasonido
int distanciaObjeto = 0;

// Define para el sensor de GPS
#define RXD2 16
#define TXD2 17

#define BUZZER_PIN 23

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Define para el sensor de velocidad
int encoder_pin = 12;
unsigned int rpm;
volatile byte pulses;
unsigned long timeold;
unsigned int pulsesperturn = 20;
const int wheel_diameter = 30;
float velocity = 0;

//Define para el sensor de infrarrojos
int IR = 34;

//Define para el zumbador
int melody[] = {
  NOTE_E6, NOTE_E7
};

int durations[] = {
  4, 4  // Duraciones en términos de la fracción de un segundo (1/4 significa una corchea)
};

int melody2[] = {
  NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A4,  // Primer tono
  NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4, NOTE_F4   // Segundo tono
};

int durations2[] = {
  8, 8, 8, 8, 8, 8, 8, 8,  // Duración del primer tono
  8, 8, 8, 8, 8, 8, 8, 8   // Duración del segundo tono
};

int melody3[] = {
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G5,
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_G5
};

int durations3[] = {
  4, 4, 4, 4, 4, 4, 4, 4,
  4, 4, 4, 4, 4, 4, 4, 4
};
int melody4[] = {
  NOTE_G4, NOTE_A4, NOTE_B4, NOTE_G4, // Parte 1
  NOTE_B4, NOTE_A4, NOTE_G4, NOTE_B4, 
  NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, // Parte 2
  NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5
};

int durations4[] = {
  4, 4, 4, 4, // Parte 1, notas con duración de 4
  8, 8, 4, 4, // Parte 1, variación en la duración
  4, 4, 4, 4, // Parte 2
  8, 8, 4, 4  // Parte 2, variación en la duración
};




const int buttonPin = 13;
const int ledRojoPin = 32;
bool buttonState = LOW;

void readUltrasonido();
void readGPS();
void readVelocimetro();
void counter() {
  pulses++;
}
void readInfrarrojos();
void zumbar();
//Creamos el scheduler
Scheduler scheduler;
//Creamos los threads

Task task_sensores(200, TASK_FOREVER, &sensores, &scheduler, true);
//Task task_button(100, TASK_FOREVER, &readButton, &scheduler, true);
Task task_infrarojos(100, TASK_FOREVER, &readInfrarrojos, &scheduler, true);
Task task_sirena(10, TASK_FOREVER, &zumbar2, &scheduler, false);  // Tarea para la sirena, inicialmente detenida


void initWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
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
      client.subscribe("sensores/respuesta");  // Suscribe a un tema si es necesario
    } else {
      Serial.println("Error en la conexión MQTT. Reintentando en 5 segundos...");
      delay(5000);
    }
  }
}

void initMQTTServer() {
  client.setServer(mqtt_server, mqtt_port);
  reconnect();
}

void sensores() {
  readGPS();
  readVelocimetro();
  readMPU();
  readUltrasonido();
}

void readUltrasonido() {
  if (estado != false) {

    unsigned int distanciaObjeto = sonar1.ping_cm();
    Serial.print(distanciaObjeto);
    Serial.println(" cm");
    if (distanciaObjeto < 15) {
      zumbar();
    }


    // Convertir el entero a String para poder publicarlo
    char buffer[10];  // Asumiendo que la distancia nunca superará 9999 cm
    snprintf(buffer, sizeof(buffer), "%u", distanciaObjeto);
  }
}

void readButton() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    estado = true;
    // turn LED on:
    digitalWrite(ledRojoPin, HIGH);
  } else {

    // turn LED off:
    estado = false;
    accidente = false;
    digitalWrite(ledRojoPin, LOW);
  }
}

void readGPS() {
  gps.encode(GPSSerial.read());
  Serial.print("Lat: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Lng: ");
  Serial.println(gps.location.lng(), 6);
  String lat = String(gps.location.lat(), 6);
  String lng = String(gps.location.lng(), 6);
  localizacion = " lat:" + lat + " lng:" + lng;

  if (estado == 0) {
    topic = "Patinete" + idPatinete + "/Localizacion";
    client.publish(topic.c_str(), localizacion.c_str());
  }
}

void readVelocimetro() {
  if (millis() - timeold >= 200) {
    detachInterrupt(digitalPinToInterrupt(encoder_pin));
    rpm = (pulses * 60) / (pulsesperturn * ((millis() - timeold) / 1000.0));
    velocity = rpm * 3.1416 * wheel_diameter * 60 / 100000;
    //Serial.print("Velocity: ");
    //Serial.println(velocity);
    String v = String(velocity, 6);
    if (estado == 1) {
      payload = localizacion + " vel:" + v;
      topic = "Patinete" + idPatinete + "/Viaje/Instante";
    }
    timeold = millis();
    pulses = 0;
    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, RISING);
  }
}

void readInfrarrojos() {

  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    // Verifica si el valor recibido es igual al valor hexadecimal deseado
    if (IrReceiver.decodedIRData.decodedRawData == 0xE31CFF00 || IrReceiver.decodedIRData.decodedRawData == 0xDC230707) {  //Boton ok del mando
      // Cambia el valor de 'estado' de true a false o de false a true
      estado = !estado;
      // Puedes imprimir el nuevo estado para verificar
      Serial.print("Estado cambiado a: ");
      Serial.println(estado ? "true" : "false");
      client.publish("sensores/estado", estado ? "true" : "false");
      zumbar();  // Función que activa un zumbador o similar
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xE916FF00 || IrReceiver.decodedIRData.decodedRawData == 0xFD020707) {  //Boton ok del mando
      //suena un timbre
      zumbar2();  // Función que activa un zumbador o similar
    }
    if (IrReceiver.decodedIRData.decodedRawData == 0xF20DFF00 || IrReceiver.decodedIRData.decodedRawData == 0xFE010707){  //Boton ok del mando
      //suena un timbre
      estadoAc = (estadoAc == 0) ? 1 : 0;

      // Cambiar los valores de umbralAceleracion y umbralRotacion
      if (estadoAc == 0) {
        umbralAceleracion = -10.0;  // Valor original de desaceleración
        umbralRotacion = 1.5; 
        zumbar3();
              // Valor original de rotación
      } else {
        umbralAceleracion = -20.0;   // Nuevo valor de desaceleración
        umbralRotacion = 6.0;  
        zumbar4();     // Nuevo valor de rotación
      }  // Función que activa un zumbador o similar
    }

    IrReceiver.resume();  // Prepara para recibir el próximo dato
  }
  if (estado == true) {


    // turn LED on:
    digitalWrite(ledRojoPin, HIGH);
  } else {
    accidente = false;
    digitalWrite(ledRojoPin, LOW);
  }
  if (accidente && estado) {
    if (!task_sirena.isEnabled()) {
      task_sirena.enable();
    }
  } else {
    if (task_sirena.isEnabled()) {
      task_sirena.disable();
    }
  }
}

void readMPU() {
  if (estado == 1) {


    /* Calcular los ángulos de Euler */
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    /* Calcular los ángulos de Euler */
    float roll = atan2(accel.acceleration.y, accel.acceleration.z) * 180 / M_PI;
    float pitch = atan(-accel.acceleration.x / sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * 180 / M_PI;
    float yaw = atan2(accel.acceleration.z, accel.acceleration.x) * 180 / M_PI;  // Esto es una aproximación

    /* Imprimir los valores */
    Serial.print("Aceleración: ");
    Serial.print(accel.acceleration.x);
    Serial.print(", ");
    Serial.print(accel.acceleration.y);
    Serial.print(", ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Giroscopio: ");
    Serial.print(gyro.gyro.x);
    Serial.print(", ");
    Serial.print(gyro.gyro.y);
    Serial.print(", ");
    Serial.print(gyro.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Ángulos de Euler: ");
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("°, Pitch: ");
    Serial.print(pitch);
    Serial.print("°, Yaw: ");
    Serial.print(yaw);
    Serial.println("°");

    float aceleracion = sqrt(accel.acceleration.x * accel.acceleration.x + accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z) / 9.81;  // Aceleración en g
    float velocidadAngular = sqrt(gyro.gyro.x * gyro.gyro.x + gyro.gyro.y * gyro.gyro.y + gyro.gyro.z * gyro.gyro.z);                                                          // Velocidad angular en rad/s

    Serial.print("Aceleración total: ");
    Serial.print(aceleracion);
    Serial.println(" g");

    Serial.print("Velocidad angular total: ");
    Serial.print(velocidadAngular);
    Serial.println(" rad/s");

    if (aceleracion < umbralAceleracion || velocidadAngular > umbralRotacion) {

      accidente = true;

      Serial.println("Accidente detectado!");

      payload = localizacion + " Accidente:" + String(accidente);
      topic = "Patinete" + idPatinete + "/Viaje/Accidente";

      // Aquí puedes agregar lógica adicional, como enviar una alerta o detener el patinete.
    }

    // Crear el payload como una cadena
    // Publicar los ángulos de Euler
    if (estado == 1) {
      client.publish(topic.c_str(), payload.c_str());
    }
  }
}


void zumbar() {
  int size = sizeof(durations) / sizeof(int);
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations[note];
    tone(BUZZER_PIN, melody[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}
void zumbar2() {
  int size = sizeof(durations2) / sizeof(int);
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations2[note];
    tone(BUZZER_PIN, melody2[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}
void zumbar3() {
  int size = sizeof(durations3) / sizeof(int);
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations3[note];
    tone(BUZZER_PIN, melody3[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}
void zumbar4() {
  int size = sizeof(durations4) / sizeof(int);
  for (int note = 0; note < size; note++) {
    int duration = 1000 / durations4[note];
    tone(BUZZER_PIN, melody4[note], duration);
    int pauseBetweenNotes = duration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER_PIN);
  }
}
void loop() {
  scheduler.execute();
}

void setup() {

  Serial.begin(115200);
  while (!Serial)
    delay(10);  // Esperar a que se inicie el monitor serie

  // Setup del wifi
  initWiFi();
  initMQTTServer();



  // Asegúrate de que el cliente MQTT está conectado
  while (!client.connected()) {
    Serial.println("No conectado al broker MQTT");
  }
  zumbar();

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

  // Pinmode Boton auxiliar
  pinMode(buttonPin, INPUT);
  pinMode(ledRojoPin, OUTPUT);


  //Setup para el sensor de velocidad
  pinMode(encoder_pin, INPUT);
  pulses = 0;
  rpm = 0;
  timeold = millis();
  attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);

  Serial.println("EMPEZAMOS");
  delay(2000);
  // Iniciar el planificador
  scheduler.startNow();
}













//Thread para ver los datos del GPS


//Thread para ver los datos del velocímetro


//Thread para ver los datos del infrarojos
