#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Configuración del PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Configuración de los límites de pulso en microsegundos
#define SERVOMIN 1000   // Pulso mínimo para 0 grados
#define SERVOMAX 2000   // Pulso máximo para 180 grados

// Dimensiones de la pata en mm
float L_hip = 31.15; // Longitud de la cadera
float a1 = 77.80;    // Longitud del fémur
float a2 = 84.86;    // Longitud de la tibia

//Offset para grados de la cadera
//float theta1_offset = 15.0;

// Función para convertir ángulo a pulso PWM
int angleToPulse(int angle) {
  int pulseUs = map(angle, 0, 180, SERVOMIN, SERVOMAX);

  // Convierte el pulso en microsegundos a un valor que el PCA9685 entienda
  int pulsePCA9685 = pulseUs * 4096 / 20000;  // Ajuste para 50 Hz (20 ms por ciclo)
  
  return pulsePCA9685;
  }

// Función de cinemática inversa
void inverseKinematics(float x, float y, float z, float &theta1, float &theta2, float &theta3) {
  // Cálculo del ángulo de la cadera (theta1)
  theta1 = atan2(y, x) * 180 / PI;

  // Distancia proyectada en el plano horizontal (excluyendo la cadera)
  float r = sqrt(x * x + y * y) - L_hip;

  // Distancia total al punto objetivo
  float d = sqrt(r * r + z * z);

  // Cálculo del ángulo del fémur (theta2)
  theta2 = atan2(z, r) * 180 / PI + acos((a1 * a1 + d * d - a2 * a2) / (2 * a1 * d)) * 180 / PI;

  // Cálculo del ángulo de la tibia (theta3)
  theta3 = 180 - acos((a1 * a1 + a2 * a2 - d * d) / (2 * a1 * a2)) * 180 / PI;

  // Diagnóstico para depuración
  Serial.print("Theta1 (Cadera): "); Serial.println(theta1);
  Serial.print("Theta2 (Fémur): "); Serial.println(theta2);
  Serial.print("Theta3 (Tibia): "); Serial.println(theta3);
}

// Función para mover una pata
void moveLeg(float x, float y, float z, int servoCadera, int servoFemur, int servoTibia) {
  float theta1, theta2, theta3;

  // Calcular ángulos de las articulaciones
  inverseKinematics(x, y, z, theta1, theta2, theta3);

  // Convertir ángulos a pulsos PWM
  int pulse1 = angleToPulse(theta1);
  int pulse2 = angleToPulse(theta2);
  int pulse3 = angleToPulse(theta3);

  // Diagnóstico para depuración
  Serial.print("Pulse1 (Cadera): "); Serial.println(pulse1);
  Serial.print("Pulse2 (Fémur): "); Serial.println(pulse2);
  Serial.print("Pulse3 (Tibia): "); Serial.println(pulse3);

  // Enviar pulsos a los servos
  pwm.setPWM(servoCadera, 0, pulse1);
  pwm.setPWM(servoFemur, 0, pulse2);
  pwm.setPWM(servoTibia, 0, pulse3);
}

void setup() {
  Serial.begin(115200);  // Inicializar comunicación serial para depuración
  pwm.begin();           // Inicializar PCA9685
  pwm.setPWMFreq(50);    // Configurar frecuencia PWM a 50 Hz

  Serial.println("Iniciando control de pata con cinemática inversa");
}

void loop() {
  float x_centro = 109.0;
  float z_centro = -20.34; // Centro del semicírculo
  float radio = 15.0;      // Radio reducido para suavidad
  int pasos = 25;          // Número de pasos para la trayectoria
  float theta1_base = 0; //Angulo base de la cadera
  float Amplitud = 35.0; //Amplitud de oscilacion 
  float omega  = 1.0; //Frecuencia de oscilacion

  //Tiempo actual en segundos
  float t = millis()/1000.0;

  // **Movimiento de ida de la pata frontal derecha y lateral izquierda**

  for (int i = 0; i <= pasos; i++) {
    float t_norm = (float)i / pasos; // Normalizar t entre 0 y 1

    // Variación dinámica de theta1
    float theta1_dynamic = theta1_base + Amplitud * sin(omega * t);
    float theta1_rad = theta1_dynamic * PI / 180.0;

    // Interpolación sinusoidal para suavidad
    float x = x_centro + radio * cos(t_norm * PI);
    float z = z_centro - radio * sin(t_norm * PI); // Montañita suave
    float y = radio * cos(t_norm * PI) * sin(theta1_rad);

    // Mueve la pata frontal derecha y lateral izquierda
    moveLeg(x, y, z, 8, 9, 10);
    moveLeg(x, y, z, 4, 5, 6);
    delay(10);
  }

  // **Movimiento de regreso de la pata frontal derecha y lateral izquierda**
  for (int i = pasos; i >= 0; i--) {
    float t_norm = (float)i / pasos; // Normalizar t entre 0 y 1

    // Variación dinámica de theta1
    float theta1_dynamic = theta1_base + Amplitud * sin(omega * t);
    float theta1_rad = theta1_dynamic * PI / 180.0;

    // Interpolación sinusoidal inversa para el regreso
    float x = x_centro + radio * cos(t_norm * PI);
    float z = z_centro - radio * sin(t_norm * PI); // Montañita suave
    float y = radio * cos(t_norm * PI) * sin(theta1_rad);

    // Mueve la pata frontal derecha y lateral izquierda
    moveLeg(x, y, z, 4, 5, 6);
    moveLeg(x, y, z, 8, 9, 10);
    delay(10);
  }

  // **Movimiento de ida de la pata frontal izquierda y lateral derecha**
  for (int i = 0; i <= pasos; i++) {
    float t_norm = (float)i / pasos; // Normalizar t entre 0 y 1

    // Variación dinámica de theta1
    float theta1_dynamic = theta1_base + Amplitud * sin(omega * t);
    float theta1_rad = theta1_dynamic * PI / 180.0;

    // Interpolación sinusoidal para suavidad
    float x = x_centro + radio * cos(t_norm * PI);
    float z = z_centro - radio * sin(t_norm * PI); // Montañita suave
    float y = radio * cos(t_norm * PI) * sin(theta1_rad);

    // Mueve la pata frontal izquierda y lateral derecha
    moveLeg(x, y, z, 0, 1, 2);
    moveLeg(x, y, z, 12, 13, 14);
    delay(10);
  }

  // **Movimiento de regreso de la pata frontal izquierda y lateral derecha**
  for (int i = pasos; i >= 0; i--) {
    float t_norm = (float)i / pasos; // Normalizar t entre 0 y 1

    // Variación dinámica de theta1
    float theta1_dynamic = theta1_base + Amplitud * sin(omega * t);
    float theta1_rad = theta1_dynamic * PI / 180.0;

    // Interpolación sinusoidal inversa para el regreso
    float x = x_centro + radio * cos(t_norm * PI);
    float z = z_centro - radio * sin(t_norm * PI); // Montañita suave
    float y = radio * cos(t_norm * PI) * sin(theta1_rad);

    // Mueve la pata frontal izquierda y lateral derecha
    moveLeg(x, y, z, 0, 1, 2);
    moveLeg(x, y, z, 12, 13, 14);
    delay(10);
  }

}
