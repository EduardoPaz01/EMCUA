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

//util 

unsigned long periodToRPM(unsigned long period_ms) {
  if (period_ms == 0) return 0; // evita divisão por zero
  double freq = 1000.0 / (double)period_ms;   // Hz
  double rpm  = freq * 60.0;                  // RPM
  return (unsigned long)rpm;
}


// temporary 

void uartInit(unsigned long baud) {
  unsigned int ubrr = F_CPU/16/baud - 1;

  // Configura baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // Habilita transmissão
  UCSR0B = (1 << TXEN0);

  // Frame format: 8 data bits, 1 stop bit, sem paridade (8N1)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uartSendChar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uartSendInt(uint16_t value) {
  char buf[6];
  int i = 0;

  do {
    buf[i++] = (value % 10) + '0';
    value /= 10;
  } while (value > 0);

  while (i > 0) {
    uartSendChar(buf[--i]);
  }

  uartSendChar('\r');
  uartSendChar('\n');
}

void sensorReadAndSend(void) {
  ADCSRA |= (1 << ADSC);            // Inicia conversão
  while (ADCSRA & (1 << ADSC));     // Espera terminar
  uint16_t adcValue = ADC;          // Lê resultado
  uartSendInt(adcValue);            // Envia pela serial
}

ISR(TIMER0_COMPA_vect) {
  g_millis++;
}

void millisInit(void) {
  // Timer0 em modo CTC, interrupção a cada 1ms
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64
  OCR0A = (F_CPU / 64 / 1000) - 1;    // 1ms
  TIMSK0 = (1 << OCIE0A);
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