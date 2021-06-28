
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

## Usage
To get use of the sound alarm and the LCD output, these should be wired. 
- A 100-150ohm resistor with a 8ohm speaker were used for the tests.
- A well as a 2x16 LCD in 8-wire configuration. (pins are showed in code as #define).
- The DAC variables that should be modified depending on your board:

`#define SCALE 0.5	// Magnitude scaling factor`  
`#define VREF 2.5	    // Used voltage reference`
 
- The default accepted passwords are:

      // Database with stored passcodes and names
    	static PASSCODE db[NUM_PASSCODES] = {
	    			{"0123", "David Calles"},
	    			{"9999", "Allan Smith"},
	    			{"0001", "Visitor1"},
	    			{"0002", "Visitor2"},
	    			{"0003", "Visitor3"},
	    			{"0004", "Visitor4"},
	    			{"0005", "Visitor5"},
	    			{"0006", "Visitor6"},
					{"0007", "Visitor7"},
	    			{"0008", "Visitor8"}
    	};


## Improvements
 Code is far from perfect. 
 Any improvement proposals are welcome.

