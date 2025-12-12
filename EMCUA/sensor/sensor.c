#include "sensor.h"

// Global sensor state
volatile unsigned long g_millis = 0;
unsigned long startMillis = 0;
uint8_t state = 0;
uint8_t firstFall = 1;
unsigned long lastPeriod = 0;
Sensor_State g_sensor;

void sensorInit(void) {
  // Configure LED pin as output
  LED_DDR |= (1 << LED_PIN);
  LED_PORT &= ~(1 << LED_PIN);  // LED off initially
  
  // Configure ADC
  ADMUX = (1 << REFS0)                    // AVCC with external capacitor at AREF pin
        | (ADC_CHANNEL & 0x0F);           // Select ADC channel
  
  ADCSRA = (1 << ADEN)              // Habilita ADC
          | (1 << ADPS0);           // Prescaler = 2 (mais rápido possível)
  
  ADCSRB = 0;                             // Free running mode
  
  // Disable digital input on ADC pin
  DIDR0 = (1 << ADC_CHANNEL);
  
  // Initialize sensor state
  g_sensor.led_state = 0;
  g_sensor.debouncing = 0;
}
