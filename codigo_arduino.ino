#include <TaskScheduler.h>

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // Crear un objeto NewPing Ultrasonido

#define TRIGGER_PIN1 26  // Pin de salida del ultrasonido (conectar al pin TRIG del sensor)
#define ECHO_PIN1 27     // Pin de entrada del ultrasonido (conectar al pin ECHO del sensor)
void readUltrasonido();

//Creamos el scheduler
Scheduler scheduler;
//Creamos los threads
Task task_botella(50, TASK_FOREVER, &readUltrasonicBotellaTask, &scheduler, true);
void setup() {

  Serial.begin(9600);
  delay(2000);

  // Iniciar el planificador
  scheduler.startNow();
}

//Thread para ver si hay botella
void readUltrasonicBotellaTask() {
  unsigned int distanciaObjeto = sonar1.ping_cm();
  serial.println(distanciaObjeto);
}
