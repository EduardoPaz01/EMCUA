#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "../uart/UART.h"

// PINNING
#define clockwise_PIN PD3        // CH5 - 5 - PIN_3
#define counterclockwise_PIN PD5 // CH5 - 6 - PIN_5

// ABSTRACTION
#define ON 1
#define OFF 0

// CLOSED-LOOP CONTROL VARIABLES
#define FREQ_HZ 1000
#define PERIOD_MS (1000/FREQ_HZ)
#define DUTY_MAX 100
#define DUTY_MIN 36
#define A_MOTOR 0.396f
#define B_MOTOR 7.17f
#define KP 0.05f
#define KI 2.0f
#define TS 0.05f

typedef struct {
  unsigned long rpm;
  uint8_t duty;
} LogData;

// GENERAL
void motorInit(void);
void motorWrite(int8_t direction, uint8_t powerMode);
void pwmInit(void);
void applyPWM(int8_t direction, uint8_t duty);

// OPEN-LOOP CONTROL
uint8_t voltageToDuty(float voltage);
void motorRamp(float v_init, float v_target, uint16_t accel_time_ms);

// CLOSED-LOOP CONTROL
float motorDiscrete(float duty);
uint8_t applyControl(unsigned long rpm_measured);
void storeValues(unsigned long rpm, uint8_t duty);
void sendLog(void);
void setRPM_REF(uint16_t ref);

// G(s)= 11,87 / 0,054s+1
// G(z)= 7,17​​  / z−0,396
// y[k] = 0,396y[k−1] + 7,17u[k−1]
// y[k] = RPM 
// u[k] = duty cicle (%)	​

#endif /* MOTOR_CONTROLLER_H_ */