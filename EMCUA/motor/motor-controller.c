#include "motor-controller.h"

void motorInit(void){
  DDRD = 0UL
    |(1<<DDD3)  // clockwise_PIN
    |(1<<DDD5); // counterclockwise_PIN
}

void motorWrite(int8_t direction, uint8_t powerMode){
  if(powerMode == ON){ // turn on
    if(direction == 1){
      PORTD |= (1<<clockwise_PIN);
    }
    else if(direction == -1){
      PORTD |= (1<<counterclockwise_PIN);
    }
  }
  else if (powerMode == OFF){ //turn off
    if(direction == 1){
      PORTD &= ~(1<<clockwise_PIN);
    }
    else if(direction == -1){
      PORTD &= ~(1<<counterclockwise_PIN);
    }
  }
}

void pwmInit(void){
  /* --- Configura portas como saída --- */
  DDRD |= (1 << PD3) | (1 << PD5);  /* PD3 = OC2B, PD5 = OC0B */

  /* -----------------------
      Timer/Counter0 (OC0B) - PD5
      - Modo Fast PWM 8-bit: WGM02:0 = 3 -> WGM02=0, WGM01=1, WGM00=1
      - COM0B1:0 = 2 -> non-inverting (Clear on compare match)
      - Prescaler CS02:0 = 011 -> clk/64
      ----------------------- */
  TCCR0A = (1 << WGM01) | (1 << WGM00)   /* Fast PWM (WGM01:0 = 11) */
          | (1 << COM0B1);                /* non-inverting on OC0B */
  TCCR0B = (0 << WGM02)                  /* WGM02 = 0 -> completes WGM = 3 */
          | (1 << CS01) | (1 << CS00);    /* CS02:0 = 0b011 -> clk/64 */

  /* Inicialmente 0% duty */
  OCR0B = 0;

  /* -----------------------
      Timer/Counter2 (OC2B) - PD3
      - Modo Fast PWM 8-bit: WGM22:0 = 3 -> WGM22=0, WGM21=1, WGM20=1
      - COM2B1:0 = 2 -> non-inverting
      - Prescaler CS22:0 = 100 -> clk/64 (Timer2 prescaler bits differ but /64 available)
      ----------------------- */
  TCCR2A = (1 << WGM21) | (1 << WGM20)   /* Fast PWM (WGM21:0 = 11) */
          | (1 << COM2B1);                /* non-inverting on OC2B */
  TCCR2B = (0 << WGM22)                  /* WGM22 = 0 -> completes WGM = 3 */
          | (1 << CS22);                 /* CS22:0 = 100 -> clk/64 for Timer2 */

  /* Inicialmente 0% duty */
  OCR2B = 0;
}
