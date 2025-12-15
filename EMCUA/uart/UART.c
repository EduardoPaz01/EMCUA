#include "UART.h"

#define CMD_BUF_SIZE 32
char cmdBuffer[CMD_BUF_SIZE];
uint8_t cmdIndex = 0;

uint8_t parseCommand(char *cmd){
  uint8_t retorno = 0;

  // SSC.1234 -> set RPM reference to 1234 (expects exactly 4 digits)
  if (strncmp(cmd, "SSC.", 4) == 0) { // Set Speed in Closed-loop
    char *p = &cmd[4];
    if (strlen(p) >= 4) {
      int valid = 1;
      for (int i = 0; i < 4; ++i) {
        if (p[i] < '0' || p[i] > '9') { valid = 0; break; }
      }
      if (valid) {
        char tmp[5];
        memcpy(tmp, p, 4);
        tmp[4] = '\0';
        setRPM_REF( atoi(tmp) );
      }
    }
    retorno = MF_RESPONSE;
  }

  // SDO.12 -> set duty cycle reference to 12
  // Accept 1 to 2 digits after the dot (e.g., SDO.0 .. SDO.10)
  else if(strncmp(cmd, "SDO.", 4) == 0){ // Set Duty-cycle in Open-loop
    char *p = &cmd[4];
    size_t len = strlen(p);
    if (len >= 1 && len <= 2) {
      int valid = 1;
      for (size_t i = 0; i < len; ++i) {
        if (!isdigit((unsigned char)p[i])) { valid = 0; break; }
      }
      if (valid) {
        char tmp[5];
        memcpy(tmp, p, len);
        tmp[len] = '\0';
        setPWM_Ref(atoi(tmp));
      }
    }
    retorno = MA_RESPONSE;
  }

  // SDS.12 -> set duty cycle step to 12
  // Accept 1 to 2 digits after the dot (e.g., SDO.0 .. SDO.10)
  else if(strncmp(cmd, "SDS.", 4) == 0){ // Set Duty-cycle in Open-loop
    char *p = &cmd[4];
    size_t len = strlen(p);
    if (len >= 1 && len <= 2) {
      int valid = 1;
      for (size_t i = 0; i < len; ++i) {
        if (!isdigit((unsigned char)p[i])) { valid = 0; break; }
      }
      if (valid) {
        char tmp[5];
        memcpy(tmp, p, len);
        tmp[len] = '\0';
        setStepPWM(atoi(tmp));
      }
    }
    retorno = MA_RESPONSE;
  }

  else {
    retorno = DEFAULT_RESPONSE;
  }

  return retorno;
}

uint8_t uartCommandTask(void){
  uint8_t retorno = 0;

  if (UCSR0A & (1 << RXC0)) {
    char c = uartReadChar();

    if (c == '\n' || c == '\r') {
      cmdBuffer[cmdIndex] = '\0';
      cmdIndex = 0;
      retorno = parseCommand(cmdBuffer);
    } 
    else if (cmdIndex < CMD_BUF_SIZE - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }

  return retorno;
}

void uartInit(unsigned long baud) {
  unsigned int ubrr = F_CPU/16/baud - 1;

  // Configure baud rate
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // Enable transmission
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);

  // Frame format: 8 data bits, 1 stop bit, without parity (8N1)
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uartSendChar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uartSendInt(uint16_t value) {
  char buf[6];
  int i = 0;

  do {
    buf[i++] = (value % 10) + '0';
    value /= 10;
  } while (value > 0);

  while (i > 0) {
    uartSendChar(buf[--i]);
  }

  uartSendChar('\r');
  uartSendChar('\n');
}

void uartEnableRX(void){
  UCSR0B |= (1 << RXEN0);
}

char uartReadChar(void){
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;
}


