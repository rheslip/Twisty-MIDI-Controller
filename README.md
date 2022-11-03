# Twisty MIDI Controller
 Arduino MIDI Controller inspired by the MIDI Fighter Twister

This controller can send four banks of 16 MIDI CC values. It also has one bank of four CC LFOs. 

CC values are read from 16 pots connected to a Arduino Pro Micro via a CD4067 sixteen channel analog mux. Pots (I used 100k) are connected between 5v and GND, wipers got to analong MUX inputs.

User interface is two momentary tact buttons and four colored LEDs. LED Anodes are connected to the PWM output ports via 1K resistors, cathodes to GND.

Left button controls CC bank - CC#s start at 16 for bank 1, 32 for bank 2, 48 for bank 3, 64 for bank 4

Bank number is shown by LED color, Red=bank 1, Orange=bank 2, Yellow=bank 3, Green=bank 4

Right button sets MIDI channel which defaults to channel 1

Press button to see flashing channel number in binary (Red is bit 0)

If right button pressed again while flashing it increments the MIDI channel number.

The filter I implemented on the A/D works great - virtually zero spurious CCs are sent - only sends new values when you move the knobs.

Libraries Used: 

MIDIUSB

FlexiTimer2


History:

10/25/22 - added LFO code from my eurorack module - implements 4 lfos on a 5th page, LEDs flash with the LFOs

4 rows of pots control 4 lfos: <rise rate> <fall rate> <waveform> <max CC value>

If right button is pressed to "shift", rightmost pot sets <min CC value>

3 waveforms: pot full left= ramp, pot in middle=pulse, pot full right=random

Didn't implement the sine etc waves from the eurorack module because the CC resolution is so coarse it would hardly be noticable

10/25/22 - swapped yellow LED pin and mux in pin. apparently pin 6 does not support PWM - maybe I have a knockoff AVR chip?
