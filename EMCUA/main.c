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

  unsigned long rpm;
  uint8_t duty;

  duty = 60;
  applyPWM(1, duty); // start Motor
  _delay_ms(100);

  char MA [16] = "MA   -  DT:     ";
  char MF [16] = "MF   -  DT:     ";
  char GR [16] = "SPEED:          "; // General 

  uartSendString("SISTEM INIT");
  
  uint8_t CL = 0;
  uint8_t OL = 1;
  while (1) {
    uint8_t response = uartCommandTask();
    if (response == MF_RESPONSE){
      CL = 1;
      OL = 0;
    }
    else if(response == MA_RESPONSE){
      CL = 0;
      OL = 1;
    }

    uint16_t val = adcRead();
    if (val < THRESHOLD) {
      if (firstFall) {
        resetMillis();
        firstFall = 0;
      } else {
        lastPeriod = millis();
        rpm = periodToRPM(lastPeriod);

        if(OL){ // Open-loop control
          applyPWM_Ref();
          duty = getDuty();
          send_LCD_label_value(MA, duty, 1);
        }
        else if(CL){ // Closed-loop control
          duty = applyControl(rpm);
          applyPWM(1, duty);
          storeValues(rpm, duty);
          send_LCD_label_value(MF, (uint16_t)duty, 1);
        }
        
        send_LCD_label_value(GR, rpm, 2);

        resetMillis();
      }
      while (adcRead() < THRESHOLD);
    }
  }
}