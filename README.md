# ATTiny24A voltage logger slave
Read the voltage of a single cell and transport the data to the master device

# PIN Assignments
Overview of used Pins

## Port A
- PA0 - ADC0    | Read cell voltage - Positive Differential Input
- PA1 - ADC1    | Read cell voltage - Negative Differential Input
- PA2 ?
- PA3 ?
- PA4 ?
- PA5 ?
- PA6 ?
- PA7 ? OSC0B   | PWM for indicator LED?

## Port B
- PB0 - XTAL    | 1.8432 MHz Crystal
- PB1 - XTAL    | 1.8432 MHz Crystal
- PB2 - INT0    | Wake up from Power-down mode (only low level will wake up the MCU) & busy indicator
- PB3 - RESET