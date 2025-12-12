#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define clockwise_PIN PD3        // CH5 - 5 - PIN_3
#define counterclockwise_PIN PD5 // CH5 - 6 - PIN_5

#define ON 1
#define OFF 0

/**
 *
 */
typedef struct {
  uint8_t direction;      // Motor direction (0: forward, 1: reverse)
  uint8_t pwm_duty1;     // PWM duty cycle for pin 1
  uint8_t pwm_duty2;     // PWM duty cycle for pin 2
  uint8_t enabled;       // Motor enable state
} Motor_Config;

/**
 *
 */
void motorInit(void);

/**
 *
 */
void motorWrite(int8_t direction, uint8_t powerMode);

void motorRamp(float v_init, float v_target, uint16_t accel_time_ms);

uint8_t voltageToDuty(float voltage);

void applyPWM(int8_t direction, uint8_t duty);

void pwmSetDutyPercent_PD3(uint8_t percent);
void pwmSetDutyPercent_PD5(uint8_t percent);

void pwmInit(void);

#endif /* MOTOR_CONTROLLER_H_ */