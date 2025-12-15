#ifndef SENSOR_H_
#define SENSOR_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// PINS AND PORTS DEFINITIONS
#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_PIN  PB0

// ADC CONFIGURATIONS
#define ADC_CHANNEL    1       // PC1 - CH2 - 5
#define ADC_THRESHOLD  512     // Half of maximum ADC value (1024/2)
#define DEBOUNCE_TIME  50      // Debounce time in milliseconds
#define THRESHOLD   10         // Value below which we consider a 'drop'

// PULSE CONTROL FOR SPEED LOGGING
volatile unsigned long g_millis;
unsigned long startMillis;
uint8_t state;                   // 0=idle, 1=waiting rise, 2=waiting second fall
uint8_t firstFall;
unsigned long lastPeriod;

typedef struct {
  uint16_t current_value;    // Current ADC reading
  uint16_t last_value;       // Previous ADC reading
  uint32_t last_trigger;     // Timestamp of last trigger
  uint8_t led_state;         // Current LED state
  uint8_t debouncing;        // Debounce flag
} Sensor_State;

/**
 * @brief Initialize sensor hardware and internal state.
 */
void sensorInit(void);

/**
 * @brief Read ADC and send the raw value over UART.
 */
void sensorReadAndSend(void);

/**
 * @brief Initialize a millisecond timer using Timer1 (CTC mode).
 */
void millisInit(void);

/**
 * @brief Get the current millisecond counter value.
 * @return Current milliseconds since `millisInit` (volatile-safe snapshot).
 */
unsigned long millis(void);

/**
 * @brief Reset the millisecond counter to zero.
 */
void resetMillis(void);

/**
 * @brief Initialize ADC peripheral for sensor readings.
 */
void adcInit(void) ;

/**
 * @brief Perform a blocking ADC conversion and return the result.
 * @return 10-bit ADC sample (0-1023).
 */
uint16_t adcRead(void);

/**
 * @brief Convert a period in milliseconds (time between pulses) to RPM.
 * @param period_ms Period in milliseconds between pulses.
 * @return Calculated RPM as unsigned long.
 */
unsigned long periodToRPM(unsigned long period_ms);

#endif /* SENSOR_H_ */
