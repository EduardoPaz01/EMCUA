#ifndef SENSOR_H_
#define SENSOR_H_

#include "../EMCUA.h"
#include <avr/io.h>
#include <util/delay.h>

// Function prototypes
void sensor_init(void);
void sensor_process(void);
uint16_t sensor_get_value(void);
void sensor_led_on(void);
void sensor_led_off(void);
uint8_t sensor_is_pulse_detected(void);

// External variables
extern Sensor_State g_sensor;

#endif /* SENSOR_H_ */
