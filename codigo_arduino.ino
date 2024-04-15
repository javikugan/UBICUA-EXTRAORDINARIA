#include <TaskScheduler.h>
#include <NewPing.h>

#define TRIGGER_PIN1 4  // Pin de salida del ultrasonido (conectar al pin TRIG del sensor)
#define ECHO_PIN1 2     // Pin de entrada del ultrasonido (conectar al pin ECHO del sensor)
#define MAX_DISTANCE 500  // Distancia máxima de medición en centímetros Ultrasonido
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);  // Crear un objeto NewPing Ultrasonido

void readUltrasonido();

//Creamos el scheduler
Scheduler scheduler;
//Creamos los threads
Task task_botella(50, TASK_FOREVER, &readUltrasonido, &scheduler, true);
void setup() {

  Serial.begin(9600);
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
