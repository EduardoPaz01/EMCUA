#include <xc.h>
#include "EMCUA.h"

int main(void){
  motorInit();

  pwmInit();

  sensorInit();

  uartInit(74880);

  millisInit();
  adcInit();

  applyPWM(1, 100);

  while (1) {
    uint16_t val = adcRead();

    if (val < THRESHOLD) { // detecta descida
      if (firstFall) {
        // primeira descida: apenas resetar
        resetMillis();
        firstFall = 0;
      } else {
        // segunda ou subsequente: calcula período
        lastPeriod = millis();
        //uartSendInt(lastPeriod);
        unsigned long rpm = periodToRPM(lastPeriod);
        //uartSendInt(lastPeriod);
        uartSendInt(rpm); 
        resetMillis();
      }

      // espera subir antes de detectar nova descida
      while (adcRead() < THRESHOLD);
    }

  }
  
}