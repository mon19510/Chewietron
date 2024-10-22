#ifndef MCPWM_CONFIG_H
#define MCPWM_CONFIG_H

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// Declaración de las funciones y configuraciones
void init_mcpwm_unit();
void move_servo(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_generator_t gen, float duty_cycle);

#endif
