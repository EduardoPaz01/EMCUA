#include <xc.h>
#include "EMCUA.h"

int main (){
  setup_LCD_serial_COM();
	uint16_t number = 0;
	char text [16] = "Velocidade:     "; 
	while(1){	
    send_LCD_label_value(text,number, 2);
    number ++;
    _delay_ms(100);
  }
}

/*
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
*/