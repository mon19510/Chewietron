#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Definir los límites de pulso para el servo
#define SERVOMIN 1000   // Pulso mínimo para 0 grados
#define SERVOMAX 2000   // Pulso máximo para 180 grados

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);  // Configura la frecuencia para los servos (50 Hz)
  Serial.println("Prueba de ángulo de servo");
  Serial.println("Ingresa un ángulo entre 0 y 180 para probar:");
}

void loop() {
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();
    angle = constrain(angle, 0, 180);  // Asegura que el ángulo esté entre 0 y 180
    pwm.setPWM(0, 0, angleToPulse(angle));
    Serial.print("Ángulo del servo: ");
    Serial.println(angle);
  }
}


int angleToPulse(int angle) {
  int pulseUs = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Convierte el pulso en microsegundos a un valor que el PCA9685 entienda
  int pulsePCA9685 = pulseUs * 4096 / 20000;  // Ajuste para 50 Hz (20 ms por ciclo)
  
  return pulsePCA9685;
}
