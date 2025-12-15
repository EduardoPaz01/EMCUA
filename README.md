# EMCUA
Embedded DC Motor Controller Using AVR

## Serial Commands

Use the UART serial interface to control the controller. Send ASCII commands terminated by CR or LF (carriage return or newline). Typical UART settings: 115200 8N1 (adjust in code with `uartInit`).

Supported commands:

- `SSC.1234` : Set Speed in Closed-loop. Exactly 4 decimal digits representing the target RPM. Example: `SSC.0450` sets target to 450 RPM.
- `SDO.12`   : Set Duty-cycle in Open-loop. 1 to 2 decimal digits (0..10 depending on your PWM scaling). Example: `SDO.75` sets duty value to 75.

Notes:

- Commands must not include extra characters other than the digits after the dot.
- Each command is processed when CR or LF is received.
- The device responds internally (see response codes in `EMCUA/uart/UART.h`).
# EMCUA
Embbbeded-DC-Motor-Controller-Using-AVR
