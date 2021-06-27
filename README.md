# PassCode-Comparator
Input a pass-code using the serial terminal and compare it from a list of accepted pass-codes. Implemented as a Finite States Machine and tested on a STM32L432KC board.

## Features 
- Check input length.
- Implemented as a Finite States Machine.
- USB-serial input-output UART2.
- 2x16 LCD output (GPIO) in 4-wire and 8-wire configurations.
- Sound alarm with a 12-bit DAC and DMA triggered by Timer2.

## Dependencies
- HAL libraries for STM-32L4XX.
- Adapted external library for LCD management.
- math.h
- stdio.h

## FSM Diagram

![FSM Diagram](https://github.com/DavidCalles/PassCode-Comparator/blob/main/FSM_Diagram.png)

## Improvements
 Code is far from perfect. 
 Any improvement proposals are welcome.
