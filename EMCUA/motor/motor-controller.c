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
