#ifndef SENSOR_H_
#define SENSOR_H_

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// Pins and ports definitions
#define LED_PORT PORTB
#define LED_DDR  DDRB
#define LED_PIN  PB0

// ADC configurations
#define ADC_CHANNEL    1       // PC1 - CH2 - 5
#define ADC_THRESHOLD  512     // Half of maximum ADC value (1024/2)
#define DEBOUNCE_TIME  50      // Debounce time in milliseconds
#define THRESHOLD   10         // valor abaixo do qual consideramos "descida"

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

void sensorInit(void);

void uartInit(unsigned long baud);

void uartSendInt(uint16_t value);

void sensorReadAndSend(void);

void millisInit(void);

unsigned long millis(void);

void resetMillis(void);

void adcInit(void) ;

uint16_t adcRead(void);

unsigned long periodToRPM(unsigned long period_ms);

void uartSendChar(char c);

#endif /* SENSOR_H_ */
