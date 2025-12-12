#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define clockwise_PIN PD3        // CH5 - 5 - PIN_3
#define counterclockwise_PIN PD5 // CH5 - 6 - PIN_5

#define FREQ_HZ 1000
#define PERIOD_MS (1000/FREQ_HZ)

#define ON 1
#define OFF 0

void motorInit(void);

void motorWrite(int8_t direction, uint8_t powerMode);

void pwmInit(void);

void applyPWM(int8_t direction, uint8_t duty);

uint8_t voltageToDuty(float voltage);

void motorRamp(float v_init, float v_target, uint16_t accel_time_ms);

#endif /* MOTOR_CONTROLLER_H_ */