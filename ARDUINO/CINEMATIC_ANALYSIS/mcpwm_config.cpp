#include "mcpwm_config.h"

// Implementación de la función para inicializar MCPWM
void init_mcpwm_unit() {
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 50;    // Frecuencia de PWM de 50 Hz
  pwm_config.cmpr_a = 0;        // Ciclo de trabajo inicial de PWM0A (0%)
  pwm_config.cmpr_b = 0;        // Ciclo de trabajo inicial de PWM0B (0%)
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;  // Actualiza el duty cycle inmediatamente

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  // Configurar para Timer 0
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);  // Configurar para Timer 1
}

// Implementación de la función para mover servos
void move_servo(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_generator_t gen, float duty_cycle) {
  mcpwm_set_duty(mcpwm_num, timer_num, gen, duty_cycle);
  mcpwm_set_duty_type(mcpwm_num, timer_num, gen, MCPWM_DUTY_MODE_0);  // Actualizar el duty cycle
}
