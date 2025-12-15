#include "LCD.h"

void send_LCD_message(uint8_t data, uint8_t RS){
	PORTD &= ~(0b00000100); // sets the enable pin low (the data pins are read on low edge)
	uint8_t tDataH = (data & (0b11111100))>>2; // delete last two data bits
	uint8_t tDataL = (data & (0b00000011))<<6; // delete first six data bits and shift right six times
	
	PORTB &= ~(0b00111111); //clear port b
	PORTB |= tDataH; // write port b
	PORTD &= ~(0b11010100); // clear port d
	
  if (RS == 1){
		PORTD |= tDataL; // write data to port d
		PORTD |= 0b000010100; // sets command to port d
	}else{
		PORTD |= tDataL; // write data to port d
		PORTD |= 0b00000100; // sets command to port d
	}
	_delay_us(100);
	
}

void send_LCD_text(char * text, uint8_t size, uint8_t line){ // sends array of characters to selected line on display

	//send_LCD_message(0x02, 0); 
	switch (line){
		case 1:
			send_LCD_message(0x02, 0); // sets cursor at the start of the first line
		break;
		
		case 2:
			send_LCD_message(0xC0, 0); // sets cursor at the start of the second line
		break;	
	}
	
	for(uint8_t i = 0; i < size; i++){
		send_LCD_message((text[i]),1);
	}
}

void send_LCD_label_value(char * label, uint16_t value, uint8_t line){
	for(uint8_t i=11; i<16; i++) label[i]=' ';

  label[14] = '0';

	if (value == 0){
		label [15] = '0';
	}
	else{
		uint8_t i = 0;
		while (value > 0){
			label[15 - i] = (value % 10) + '0';
			value /= 10;
			i++;
		}		 
	}
	
	send_LCD_text(label,16,line);
}

void clear_LCD(){
	send_LCD_message(0x01, 0); // resets cursor position
	send_LCD_message(0x06, 0); // clear display
}

void setup_LCD_serial_COM(){
	DDRB |= (1<<DDB0)|(1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB4)|(1<<DDB5); // define from PB0 to PB5 as output
	DDRD |= (1<<DDD2)|(1<<DDD4)|(1<<DDD6)|(1<<DDD7); // define PD2,PD4,PD6,PD7 as output
	
	send_LCD_message(0x38, 0); // sets display mode as 8 bits three times PS: 
	send_LCD_message(0x38, 0);
	send_LCD_message(0x38, 0);
	send_LCD_message(0x0E, 0); // sets cursor to underline and blink
	send_LCD_message(0x01, 0); // clear display
	send_LCD_message(0x02, 0); 
	send_LCD_message(0x06, 0); 
}