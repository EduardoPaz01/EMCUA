#pragma once

#include <stdint.h>
#include <xc.h>

/**
 * Sends a instruction or character to the Display 
 * 
 * Display data can be passed as char and instructions have an assosiated Hex number
 * 
 * RS = 1 - write to Data Register 
 * RS = 2 - write to Instruction register
 */
void send_LCD_message(uint8_t data, uint8_t RS);

/**
 * Sends an array of characters to the selected line on display
 * 
 * |the display is 16x2 and no formating is aplied|
 * |PS: I DID NOT TEST FOR EDGE CASES SO CERTIFY NO INDEXING ERRORS WILL HAPPEN|
 */
void send_LCD_text(char * text, uint8_t size, uint8_t line);

/**
 * Sends and array of text to the display and overrides the converted value 
 * at the last five characters
 * 
 * |the display is 16x2 and no formating is aplied|
 * |PS: I DID NOT TEST FOR EDGE CASES SO CERTIFY NO INDEXING ERRORS WILL HAPPEN|
 */
void send_LCD_label_value(char * label, uint16_t value, uint8_t line);

/**
 * Removes all text and repositions the cursor to the top right corner
 * 
 * 
 */
void clear_LCD();

/**
 * Sets pins on the atmega 328p as write and sends and does the initial configuration of
 * the display via instruction messages
 * 
 */
void setup_LCD_serial_COM();
