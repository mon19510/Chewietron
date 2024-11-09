#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Parámetros de la estructura de la pierna
const float L_hip = 31.15;  // Longitud de la cadera (en mm)
const float a1 = 63.25;     // Longitud del fémur (en mm)
const float a2 = 74.80;     // Longitud de la tibia (en mm)

// Rango de ángulos permitido para cada articulación
// Para patas 1, 3 y caderas de patas 2 y 4
const int hipMin = 60;
const int hipMax = 75;
const int femurMin13 = 75;
const int femurMax13 = 90;
const int tibiaMin13 = 90;
const int tibiaMax13 = 110;

// Para el fémur y la tibia de las patas 2 y 4 (movimiento inverso)
const int femurMin24 = 75;
const int femurMax24 = 90;
const int tibiaMin24 = 90;
const int tibiaMax24 = 110;

// Definir los pines del PCA9685 para cada pata
// Patas 1 y 3
const int hipPin1 = 0, femurPin1 = 1, tibiaPin1 = 2;  // Pata 1
const int hipPin3 = 4, femurPin3 = 5, tibiaPin3 = 6;  // Pata 3
// Patas 2 y 4
const int hipPin2 = 8, femurPin2 = 9, tibiaPin2 = 10;  // Pata 2
const int hipPin4 = 12, femurPin4 = 13, tibiaPin4 = 14; // Pata 4

// Función para convertir ángulo a pulso de servo
#define SERVOMIN 1000
#define SERVOMAX 2000
int angleToPulse(int angle) {
  int pulseUs = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Convierte el pulso en microsegundos a un valor que el PCA9685 entienda
  int pulsePCA9685 = pulseUs * 4096 / 20000; 
  
  return pulsePCA9685;}

// Variables para almacenar los ángulos actuales de cada pata
int hipAngle1, femurAngle1, tibiaAngle1;
int hipAngle2, femurAngle2, tibiaAngle2;
int hipAngle3, femurAngle3, tibiaAngle3;
int hipAngle4, femurAngle4, tibiaAngle4;

// Función para calcular ángulos con cinemática inversa para las caderas de todas las patas
void calculateHipAngle(float x, float y, int &hipAngle) {
  float adjustedX = x - L_hip;
  hipAngle = atan2(y, adjustedX) * 180.0 / M_PI;
  hipAngle = constrain(hipAngle, hipMin, hipMax);
}

// Función para calcular ángulos con cinemática inversa para las patas 1 y 3
void calculateAngles(float x, float y, int &hipAngle, int &femurAngle, int &tibiaAngle, int femurMin, int femurMax, int tibiaMin, int tibiaMax) {
  float adjustedX = x - L_hip;
  float r = sqrt(adjustedX * adjustedX + y * y);

  // Ángulo del fémur
  float angleFemurRad = acos((r * r + a1 * a1 - a2 * a2) / (2 * r * a1));
  femurAngle = angleFemurRad * 180.0 / M_PI;
  femurAngle = constrain(femurAngle, femurMin, femurMax);

  // Ángulo de la tibia
  float angleTibiaRad = acos((a1 * a1 + a2 * a2 - r * r) / (2 * a1 * a2));
  tibiaAngle = 180.0 - (angleTibiaRad * 180.0 / M_PI);
  tibiaAngle = constrain(tibiaAngle, tibiaMin, tibiaMax);
}

// Función para calcular ángulos de fémur y tibia de las patas 2 y 4 con inversión parcial
void calculateAnglesInverted(float x, float y, int &femurAngle, int &tibiaAngle) {
  int dummyHip;  // No necesitamos calcular el ángulo de la cadera aquí
  calculateAngles(x, y, dummyHip, femurAngle, tibiaAngle, femurMin24, femurMax24, tibiaMin24, tibiaMax24);

  // Invertimos los ángulos del fémur y la tibia para el movimiento contrario
  femurAngle = femurMax24 - (femurAngle - femurMin24);
  tibiaAngle = tibiaMax24 - (tibiaAngle - tibiaMin24);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);  /
}

// Variables para definir la trayectoria de marcha para patas 1, 3 y 2, 4
const int numPuntos = 4;
// Trayectoria para patas 1 y 3
float trayectoria13[numPuntos][2] = {
  {L_hip + 30, 20},   // Pie hacia atrás y en el suelo
  {L_hip + 30, 60},   // Levanta el pie
  {L_hip + 60, 60},   // Avanza con el pie levantado
  {L_hip + 60, 20}    // Pie hacia adelante y en el suelo
};
// Trayectoria inversa para las patas 2 y 4
float trayectoria24[numPuntos][2] = {
  {L_hip + 60, 20},   // Pie hacia adelante y en el suelo (inverso)
  {L_hip + 60, 60},   // Levanta el pie (inverso)
  {L_hip + 30, 60},   // Retrae con el pie levantado (inverso)
  {L_hip + 30, 20}    // Pie hacia atrás y en el suelo (inverso)
};

int puntoActual = 0;

void loop() {
  // Obtener el punto actual de la trayectoria para patas 1 y 3
  float targetX13 = trayectoria13[puntoActual][0];
  float targetY13 = trayectoria13[puntoActual][1];

  // Obtener el punto actual de la trayectoria para las patas 2 y 4
  float targetX24 = trayectoria24[puntoActual][0];
  float targetY24 = trayectoria24[puntoActual][1];

  // Calcula los ángulos de la cadera para todas las patas (sin inversión)
  calculateHipAngle(targetX13, targetY13, hipAngle1);
  calculateHipAngle(targetX13, targetY13, hipAngle3);
  calculateHipAngle(targetX13, targetY13, hipAngle2);
  calculateHipAngle(targetX13, targetY13, hipAngle4);

  // Calcula los ángulos de fémur y tibia para las patas 1 y 3
  calculateAngles(targetX13, targetY13, hipAngle1, femurAngle1, tibiaAngle1, femurMin13, femurMax13, tibiaMin13, tibiaMax13);
  calculateAngles(targetX13, targetY13, hipAngle3, femurAngle3, tibiaAngle3, femurMin13, femurMax13, tibiaMin13, tibiaMax13);

  // Calcula los ángulos de fémur y tibia para las patas 2 y 4 con inversión
  calculateAnglesInverted(targetX24, targetY24, femurAngle2, tibiaAngle2);
  calculateAnglesInverted(targetX24, targetY24, femurAngle4, tibiaAngle4);

  // Mueve los servos de las patas 1, 2, 3 y 4 a los ángulos calculados
  pwm.setPWM(hipPin1, 0, angleToPulse(hipAngle1));  // Cadera de la pata 1
  pwm.setPWM(femurPin1, 0, angleToPulse(femurAngle1));  // Fémur de la pata 1
  pwm.setPWM(tibiaPin1, 0, angleToPulse(tibiaAngle1));  // Tibia de la pata 1

  pwm.setPWM(hipPin2, 0, angleToPulse(hipAngle2));  // Cadera de la pata 2 (igual a la de patas 1 y 3)
  pwm.setPWM(femurPin2, 0, angleToPulse(femurAngle2));  // Fémur de la pata 2 (invertido)
  pwm.setPWM(tibiaPin2, 0, angleToPulse(tibiaAngle2));  // Tibia de la pata 2 (invertido)

  pwm.setPWM(hipPin3, 0, angleToPulse(hipAngle3));  // Cadera de la pata 3
  pwm.setPWM(femurPin3, 0, angleToPulse(femurAngle3));  // Fémur de la pata 3
  pwm.setPWM(tibiaPin3, 0, angleToPulse(tibiaAngle3));  // Tibia de la pata 3

  pwm.setPWM(hipPin4, 0, angleToPulse(hipAngle4));  // Cadera de la pata 4 (igual a la de patas 1 y 3)
  pwm.setPWM(femurPin4, 0, angleToPulse(femurAngle4));  // Fémur de la pata 4 (invertido)
  pwm.setPWM(tibiaPin4, 0, angleToPulse(tibiaAngle4));  // Tibia de la pata 4 (invertido)

  // Imprime los ángulos en el monitor serial para depuración
  Serial.print("Pata 1 - Hip Angle: "); Serial.println(hipAngle1);
  Serial.print("Pata 2 - Hip Angle: "); Serial.println(hipAngle2);
  Serial.print("Pata 3 - Hip Angle: "); Serial.println(hipAngle3);
  Serial.print("Pata 4 - Hip Angle: "); Serial.println(hipAngle4);

  // Avanza al siguiente punto de la trayectoria
  puntoActual = (puntoActual + 1) % numPuntos;

  delay(500);  // Ajusta el delay para controlar la velocidad del movimiento
}
