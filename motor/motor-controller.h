#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include "../EMCUA.h"
#include <avr/io.h>
#include <util/delay.h>

typedef struct {
  uint8_t direction;      // Motor direction (0: forward, 1: reverse)
  uint8_t pwm_duty1;     // PWM duty cycle for pin 1
  uint8_t pwm_duty2;     // PWM duty cycle for pin 2
  uint8_t enabled;       // Motor enable state
} Motor_Config;

#endif /* MOTOR_CONTROLLER_H_ */