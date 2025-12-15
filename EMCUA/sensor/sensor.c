#include "sensor.h"

// GLOBAL SENSOR STATE
volatile unsigned long g_millis = 0;
unsigned long startMillis = 0;
uint8_t state = 0;
uint8_t firstFall = 1;
unsigned long lastPeriod = 0;
Sensor_State g_sensor;

ISR(TIMER1_COMPA_vect) {
  g_millis++;
}

void sensorInit(void) {
  // Configure LED pin as output
  LED_DDR |= (1 << LED_PIN);
  LED_PORT &= ~(1 << LED_PIN);  // LED off initially
  
  // Configure ADC
  ADMUX = (1 << REFS0)                    // AVCC with external capacitor at AREF pin
          |(ADC_CHANNEL & 0x0F);          // Select ADC channel
  
  ADCSRA = (1 << ADEN)                    // Habilita ADC
           |(1 << ADPS0);                 // Prescaler = 2 (mais rápido possível)
  
  ADCSRB = 0;                             // Free running mode
  
  // Disable digital input on ADC pin
  DIDR0 = (1 << ADC_CHANNEL);
  
  // Initialize sensor state
  g_sensor.led_state = 0;
  g_sensor.debouncing = 0;
}

void adcInit(void) {
  ADMUX = (1 << REFS0) | (ADC_CHANNEL & 0x0F);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // prescaler 64
  DIDR0 = (1 << ADC_CHANNEL);
}

uint16_t adcRead(void) {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

unsigned long periodToRPM(unsigned long period_ms) {
  if (period_ms == 0) return 0;               // avoids division by zero
  double freq = 1000.0 / (double)period_ms;   // Hz
  double rpm  = freq * 60.0;                  // RPM
  return (unsigned long)rpm;
}

void sensorReadAndSend(void) {
  ADCSRA |= (1 << ADSC);            // Starts conversion
  while (ADCSRA & (1 << ADSC));     // Wanting ending
  uint16_t adcValue = ADC;          // Read
  uartSendInt(adcValue);            // Send to serial
}

void millisInit(void) {
  // Timer1 in mode CTC, interrupt 1ms
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64
  OCR1A = (F_CPU / 64 / 1000) - 1;    // 1ms
  TIMSK1 = (1 << OCIE1A);
  sei();
}

unsigned long millis(void) {
  unsigned long m;
  cli();
  m = g_millis;
  sei();
  return m;
}

void resetMillis(void) {
  cli();
  g_millis = 0;
  sei();
}