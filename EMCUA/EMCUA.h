#ifndef EMCUA_H_
#define EMCUA_H_

#define F_CPU 16000000UL
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "sensor/sensor.h"
#include "motor/motor-controller.h"

#include "ihm/LCD.h"

// ADC Configuration
//#define ADC_CHANNEL 0  // ADC0 on PORTC0
//#define ADC_REFERENCE REFS0  // AVCC with external capacitor at AREF pin

// Timer Configuration
//#define TIMER1_PRESCALER 64
//#define TIMER1_TOP 24999  // For 10ms interval at 16MHz

// PWM Configuration
//#define PWM_PIN1 PB1  // OC1A
//#define PWM_PIN2 PB2  // OC1B
//#define PWM_TOP 255   // 8-bit PWM resolution


// Global variables declarations
//extern ADC_Config g_adc;
//extern Motor_Config g_motor;

#endif /* EMCUA_H_ */