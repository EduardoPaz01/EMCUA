#include "EMCUA.h"
#include "human-Interface/EMCUA_LCD.h"


int main (){
  setup_LCD_serial_COM();
	uint16_t number = 0;
	char text [16] = "Velocidade:     "; 
	while(1)
    {	
		send_LCD_label_value(text,number, 2);
		number ++;
		_delay_ms(100);
    }
}