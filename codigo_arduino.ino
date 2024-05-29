#include <TaskScheduler.h>
#include <NewPing.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define para el sensor de ultrasonido
#define TRIGGER_PIN1 4  // Pin de salida del ultrasonido (conectar al pin TRIG del sensor)
#define ECHO_PIN1 2     // Pin de entrada del ultrasonido (conectar al pin ECHO del sensor)
#define MAX_DISTANCE 500  // Distancia máxima de medición en centímetros Ultrasonido
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // Crear un objeto NewPing Ultrasonido

// Define para el sensor de GPS
#define RXD2 16
#define TXD2 17
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Define para el sensor de velocidad
int encoder_pin = 22;
unsigned int rpm;
volatile byte pulses;
unsigned long timeold;
unsigned int pulsesperturn = 20;
const int wheel_diameter = 2;
float velocity = 0;

//Define para el sensor de infrarrojos

void readUltrasonido();
void readGPS();
void readVelocimetro();
void counter(){
  pulses++;
}
void readInfrarrojos();

//Creamos el scheduler
Scheduler scheduler;
//Creamos los threads
Task task_ultrasonido(500, TASK_FOREVER, &readUltrasonido, &scheduler, true);
Task task_gps(2000, TASK_FOREVER, &readGPS, &scheduler, true);
Task task_velocimetro(50, TASK_FOREVER, &readVelocimetro, &scheduler, true);
Task task_infrarrojos(10, TASK_FOREVER, &readInfrarrojos, &scheduler, true);
void setup() {

  Serial.begin(115200);
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
  unsigned int distanciaObjeto = sonar1.ping_cm();
  Serial.print(distanciaObjeto);
  Serial.println(" cm");
}

//Thread para ver los datos del GPS
void readGPS() {
  gps.encode(GPSSerial.read());
  Serial.print("Lat: ");
  Serial.println(gps.location.lat(), 6);
  Serial.print("Lng: ");
  Serial.println(gps.location.lng(), 6);
}

//Thread para ver los datos del velocímetro
void readVelocimetro() {
  if (millis() - timeold >= 1000){
    detachInterrupt(digitalPinToInterrupt(encoder_pin));
    rpm = (pulses * 60) / ((millis() - timeold) / 1000 * pulsesperturn);
    velocity = rpm * 3.1416 * wheel_diameter * 60 / 1000000;
    Serial.print("Velocity: ");
    Serial.println(velocity);
    timeold = millis();
    pulses = 0;
    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, RISING);
  }
}

//Thread para ver los datos del infrarojos
void readInfrarrojos() {
}
