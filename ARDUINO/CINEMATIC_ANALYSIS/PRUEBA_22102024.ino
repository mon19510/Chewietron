#include "mcpwm_config.h"  // Incluir el archivo auxiliar

void setup() {
  Serial.begin(115200);

  // Inicializar el MCPWM desde el archivo auxiliar
  init_mcpwm_unit();
}

void loop() {
  // Ejemplo de movimiento de servos desde el archivo auxiliar
  move_servo(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 7.5);  // Mueve el servo al centro
  delay(1000);
}
