#include "motor-controller.h"

// CLOSED-LOOP CONTROL
LogData logBuffer[64];
float y_rpm = 0.0f;
float y_rpm_prev = 0.0f;
float u = 0.0f;
float u_prev = 0.0f;
float e = 0.0f;
float e_prev = 0.0f;
uint8_t logIndex = 0;
uint16_t rpm_ref = 400;

// GENERAL
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
  /* -----------------------
      Timer/Counter0 (OC0B) - PD5
        Mode Fast PWM 8-bit: WGM02:0 = 3 -> WGM02=0, WGM01=1, WGM00=1
        COM0B1:0 = 2 -> non-inverting (Clear on compare match)
        Prescaler CS02:0 = 011 -> clk/64
      ----------------------- */
  TCCR0A = (1 << WGM01) | (1 << WGM00)    // Fast PWM (WGM01:0 = 11) 
           |(1 << COM0B1);                // non-inverting on OC0B 
  TCCR0B = (0 << WGM02)                   // WGM02 = 0 -> completes WGM = 3 
           |(1 << CS01) | (1 << CS00);    // CS02:0 = 0b011 -> clk/64

  // 0% duty
  OCR0B = 0;

  /* -----------------------
      Timer/Counter2 (OC2B) - PD3
        Mode Fast PWM 8-bit: WGM22:0 = 3 -> WGM22=0, WGM21=1, WGM20=1
        COM2B1:0 = 2 -> non-inverting
        Prescaler CS22:0 = 100 -> clk/64 (Timer2 prescaler bits differ but /64 available)
      ----------------------- */
  TCCR2A = (1 << WGM21) | (1 << WGM20)    // Fast PWM (WGM21:0 = 11)
           |(1 << COM2B1);                // non-inverting on OC2B
  TCCR2B = (0 << WGM22)                   // WGM22 = 0 -> completes WGM = 3
           |(1 << CS22);                  // CS22:0 = 100 -> clk/64 for Timer2

  // 0% duty
  OCR2B = 0;
}
void applyPWM(int8_t direction, uint8_t duty){
  if (duty > DUTY_MAX) duty = DUTY_MAX;
  if (duty < DUTY_MIN) duty = DUTY_MIN;
  uint8_t value = (uint8_t)((duty * 255UL) / 100UL);

  uint8_t sreg = SREG;
  cli();

  if (direction == 1) {
    /* Enable OC2B (non-inverting) and disconnect OC0B */
    TCCR2A = (TCCR2A & ~((1<<COM2B1)|(1<<COM2B0))) | (1<<COM2B1); // OC2B non-inverting
    TCCR0A &= ~((1<<COM0B1)|(1<<COM0B0));                         // disconnect OC0B

    OCR2B = value;
    OCR0B = 0;
  }
  else if (direction == -1) {
    /* Enable OC0B (non-inverting) and disconnect OC2B */
    TCCR0A = (TCCR0A & ~((1<<COM0B1)|(1<<COM0B0))) | (1<<COM0B1); // OC0B non-inverting
    TCCR2A &= ~((1<<COM2B1)|(1<<COM2B0));                         // disconnect OC2B

    OCR0B = value;
    OCR2B = 0;
  }
  else {
    // direction == 0
    TCCR0A &= ~((1<<COM0B1)|(1<<COM0B0));
    TCCR2A &= ~((1<<COM2B1)|(1<<COM2B0));
    OCR0B = OCR2B = 0;
  }

  SREG = sreg;
}

// OPEN-LOOP CONTROL
uint8_t voltageToDuty(float voltage){
  if(voltage > 12.0) voltage = 12.0;
  if(voltage < -12.0) voltage = -12.0;

  // Duty cycle (0-255)
  float duty = (fabs(voltage) / 12.0) * 255.0;
  return (uint8_t)duty;
}
void motorRamp(float v_init, float v_target, uint16_t accel_time_ms){
  int steps = accel_time_ms / PERIOD_MS;
  float slope = (v_target - v_init) / steps;
  float v_current = v_init;

  for(int i=0; i<=steps; i++){
    int8_t direction = (v_current >= 0) ? 1 : -1;
    uint8_t duty = voltageToDuty(v_current);

    applyPWM(direction, duty);

    v_current += slope;
    _delay_ms(PERIOD_MS);
  }
}

// CLOSED-LOOP CONTROL
float motorDiscrete(float duty){
  float y;

  y = A_MOTOR * y_rpm_prev + B_MOTOR * duty;
  y_rpm_prev = y;

  return y;
}
uint8_t applyControl(unsigned long rpm_measured){
  y_rpm = (float)rpm_measured;

  // Error
  e = rpm_ref  - y_rpm;

  // PI 
  u = u_prev 
      + KP * (e - e_prev) 
      + KI * TS * e;

  // Saturation (anti wind-up)
  if (u > DUTY_MAX) u = DUTY_MAX;
  if (u < DUTY_MIN) u = DUTY_MIN;

  // Update memory
  u_prev = u;
  e_prev = e;

  return (uint8_t)u;
}
void storeValues(unsigned long rpm, uint8_t duty){
  if (logIndex < 64) {
    logBuffer[logIndex].rpm = rpm;
    logBuffer[logIndex].duty = duty;
    logIndex++;
  }
}

void setRPM_REF(uint16_t ref){
  rpm_ref = ref;
}
