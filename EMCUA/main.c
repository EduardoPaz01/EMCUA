#include <xc.h>
#include "EMCUA.h"

int main(void){
  motorInit();
  pwmInit();
  sensorInit();
  millisInit();
  adcInit();
  uartInit(74880);
  setup_LCD_serial_COM();
  
  applyPWM(1, 60); // start Motor
  _delay_ms(100);

  unsigned long rpm;
  uint8_t duty;

  char MA [16] = "MA   -  DT:     ";
  char MF [16] = "MF   -  DT:     ";
  char GR [16] = "SPEED:          "; // General response
  while (1) {
    uint8_t response = uartCommandTask();

    uint16_t val = adcRead();
    if (val < THRESHOLD) {
      if (firstFall) {
        resetMillis();
        firstFall = 0;
      } else {
        lastPeriod = millis();
        rpm = periodToRPM(lastPeriod);

        if(response == MA_RESPONSE){ // Open-loop control
          send_LCD_label_value(MA, rpm, 1);
        }
        else if(response == MF_RESPONSE){ // Closed-loop control
          duty = applyControl(rpm);
          applyPWM(1, duty);
          storeValues(rpm, duty);
          uartSendInt(rpm);
          send_LCD_label_value(MF, rpm, 1);
        }
        
        send_LCD_label_value(GR, rpm, 2);

        resetMillis();
      }
      while (adcRead() < THRESHOLD);
    }
  }
}