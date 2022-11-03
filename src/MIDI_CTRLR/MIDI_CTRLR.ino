/*
 * MIDIUSB_loop.ino
 *
 * Created: 4/6/2015 10:47:08 AM
 * Author: gurbrinder grewal
 * Modified by Arduino LLC (2015)
 */

// R Heslip Oct 2022 - modded the example code into a simple CC controller
// CC values are read from 16 pots connected to the Arduino Pro Mini via a CD4067 sixteen channel analog mux
// left button controls cc page - cc#s start at 16 for page 1, 32 for page 2, 48 for page 3, 64 for page 4
// page number is shown by LED color, Red=page 1, Orange=Page 2, Yellow=Page 3, Green=page 4
// right button sets MIDI channel which defaults to channel 1
// press button to see flashing channel number in binary (Red is bit 0)
// if right button pressed again while flashing it increments the channel number
// the filter I implemented on the A/D works great - virtually zero spurious CCs are sent. only sends new values when you move the knobs

// 10/25/22 - added LFO code from my eurorack module - implements 4 lfos on a 5th page, LEDs flash with the LFOs
// 4 rows of pots control 4 lfos: <rise rate> <fall rate> <waveform> <max CC value>
// if right button is pressed for "shift" parameters, rightmost pot sets <min CC value>
// 3 waveforms: pot full left= ramp, pot in middle=pulse, pot full right=random
// didn't implement the sine etc waves from the eurorack module because the CC resolution is so coarse it would hardly be noticable
// 10/25/22 - swapped yellow LED pin and mux in pin. apparently pin 6 does not support PWM

#include "MIDIUSB.h"
#include <FlexiTimer2.h>

#define NPOTS 16 // number of pots
uint16_t pot[NPOTS]; // pot readings
uint16_t lastpots[NPOTS]; // old pot readings
bool potlock[NPOTS]; // when pots are locked it means they must change by MIN_POT_CHANGE to register
uint8_t CC[NPOTS]; // CC values
uint8_t channel=0; // MIDI channel - starts at 0
uint8_t page=0; // CC page 0-3
uint8_t leds=0;
long button2timer;
bool shift=0;  // alt button mode
int mode;      // CC message mode or LFO mode
#define CCMODE 0
#define LFOMODE 1

#define RED_PIN 3 // LED output pins
#define ORANGE_PIN 5
#define YELLOW_PIN 10
#define GREEN_PIN 9
#define RED 1   // LED bit masks
#define ORANGE 2
#define YELLOW 4
#define GREEN 8
#define LEDFLASH 100 // flash rate
#define BUTTON1 4  // change bank button
#define BUTTON2 7  // change channel button
#define BUTTON2_TIME 2000 // increment timer
#define DEBOUNCE 20 // button debounce time in ms

#define MUX_IN A7 // port we read values from MUX
#define BASE_CC 16  // lowest CC number to use

// LFO related stuff - not well integrated into the other code yet because I lifted it from my eurorack LFO module

#define MIN_POT_CHANGE 25 // pot reading must change by this in order to register
#define POT_AVERAGING 5 //average to use in LFO page 
// maximum/minimum increments for the DDS which set the fastest/slowest LFO rates
//#define MAX_DELTA 2047 // max DDS ramp increment 
//#define RANGE 162  // 1024/log(MAX_DELTA) 8khz sampling
//#define RANGE 142  // 1024/log(MAX_DELTA) for 1khz sampling. gives max LFO rate about 30hz
#define RANGE 155  // 1024/log(MAX_DELTA) for 1khz sampling. gives max LFO rate about 3hz. don't want to go too fast or you will overload the midi channel
#define MIN_DELTA 1000 // minimum value for DDS adder - avoids rediculously slow ramps

#define RAMP 0 // wave shape indexes
#define PULSE 1
#define RANDOM1 2

#define WDIV 342 // divide pot reading by this to get wave shape index 0-2

struct lfodata {
  byte wave; // waveform
  long rate1; // rate for first section of waveform
  long rate2; // rate for second section of waveform
//  uint16_t amplitude;
  uint8_t maximum; // max CC value
  uint8_t minimum; // min CC value
  long acc; // bottom 20 are accumulator for DDS algorithm, top 12 used for waveform generation
  bool phase; // flags first or second section of waveform
  unsigned int dacout; // current DAC output value
//  unsigned int scaledout; // scaled output 0-1023
  unsigned int lastCC; // last CC value sent
  } lfo[4];

// waveform generator lifted from my 4lfo eurorack module
// basic waveform is a ramp - up ramp time set by one pot, down ramp time by the second
// ramp implemented with DDS algorithm. 24 bit accumulator - top 12 bits are the ramp and low 16 are the fractional increment
// other waveforms are derived from the ramp values (0-4097)

void lfo_generate() {

  bool phasechange; // indicates ramp has changed direction
  unsigned a,x,y,delta,out; 
  unsigned char i;
  
  for (i=0; i<4; ++i) {
    phasechange=0;  
    
    if (lfo[i].phase == 0) lfo[i].acc+=lfo[i].rate1; // ramp up for first part
        else lfo[i].acc-=lfo[i].rate2; // ramp down for second part

    if (lfo[i].acc >= 0x0fffffff) {  // test for accumulator overflow
      lfo[i].phase = 1 ;// ramp down once we hit the top
      lfo[i].acc=0x0fffffff; // set to max for next DAC output
      phasechange=1;
    }
    if (lfo[i].acc <=0) { // test for accumulator underflow
      lfo[i].phase = 0; //ramp up when we hit bottom
      lfo[i].acc=0; // set to 0 min for start of up ramp
      phasechange=1;
    }

    switch (lfo[i].wave) {
      case RAMP:
        lfo[i].dacout=lfo[i].acc >> 16; // get top 16 bits of accumulator. low 12 bits go to DAC
        break;    
      case PULSE: // variable duty cycle pulse
        if (lfo[i].phase) lfo[i].dacout=4095;
        else lfo[i].dacout=0;
        break;
      case RANDOM1:  // random value every time we hit max or min ramp
        if (phasechange) lfo[i].dacout=random(4096);
        break;
     }

  // scale the output
//    long scaledout=((long)lfo[i].dacout*(long)lfo[i].amplitude)/1024; // use long int math here or it will overflow
//    lfo[i].scaledout=(unsigned) scaledout; // save it for LED display
  }

} 

  // midi related stuff
// First parameter is the event type (0x09 = note on, 0x08 = note off).
// Second parameter is note-on/note-off, combined with the channel.
// Channel can be anything between 0-15. Typically reported to the user as 1-16.
// Third parameter is the note number (48 = middle C).
// Fourth parameter is the velocity (64 = normal, 127 = fastest).

void noteOn(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOn = {0x09, 0x90 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOn);
}

void noteOff(byte channel, byte pitch, byte velocity) {
  midiEventPacket_t noteOff = {0x08, 0x80 | channel, pitch, velocity};
  MidiUSB.sendMIDI(noteOff);
}

// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).

void controlChange(byte channel, byte control, byte value) {
  midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
  MidiUSB.sendMIDI(event);
}


// set analog mux address for reading pots
void muxaddr(int addr) {
  digitalWrite(A0, addr & 1);
  digitalWrite(A1, addr & 2);
  digitalWrite(A2, addr & 4);
  digitalWrite(A3, addr & 8);  
}

// flag all pot values as locked ie they have to change more than MIN_POT_CHANGE to register
void lockpots(void) {
  for (int i=0; i<NPOTS;++i) potlock[i]=1;
}

// sample analog pot input and do filtering. 
// if pots are locked, change value only if movement is greater than MIN_POT_CHANGE
uint16_t sample_pot(uint8_t potnum) {
    int val=0;
    muxaddr(potnum);
    for (int j=0; j<POT_AVERAGING;++j) val+=analogRead(MUX_IN); // read the A/D a few times and average for a more stable value
    val=val/POT_AVERAGING;
    if (potlock[potnum]) {
      int delta=lastpots[potnum]-val;  // this needs to be done outside of the abs() function - see arduino abs() docs
      if (abs(delta) > MIN_POT_CHANGE) {
        potlock[potnum]=0;   // flag pot no longer locked
        pot[potnum]=lastpots[potnum]=val; // save the new reading
      }
      else val=lastpots[potnum];
    }
    else {
      if (abs(lastpots[potnum]-val) > 7) lastpots[potnum]=val; // even if pot is unlocked, make sure pot has moved at least 8 counts so CC values don't jump around
      else val=lastpots[potnum];
      pot[potnum]=val; // pot is unlocked so save the reading
    }
    return val;
}

// set LEDs state based on bit mask
void setleds(uint8_t setting) {
  if (setting &1) digitalWrite(RED_PIN, 1);
  else digitalWrite(RED_PIN, 0); 
  if (setting &2) digitalWrite(ORANGE_PIN, 1);
  else digitalWrite(ORANGE_PIN, 0); 
  if (setting &4) digitalWrite(YELLOW_PIN, 1);
  else digitalWrite(YELLOW_PIN, 0); 
  if (setting &8) digitalWrite(GREEN_PIN, 1);
  else digitalWrite(GREEN_PIN, 0); 
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, OUTPUT);    // mux addresses
  pinMode(A1, OUTPUT);  
  pinMode(A2, OUTPUT);  
  pinMode(A3, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);  
  pinMode(BUTTON2, INPUT_PULLUP);    
  pinMode(RED_PIN, OUTPUT);  
  pinMode(GREEN_PIN, OUTPUT);  
  pinMode(ORANGE_PIN, OUTPUT);  
  pinMode(YELLOW_PIN, OUTPUT); 
  setleds(1<<page);  // show CC page number using LED - red=1, orange=2, yellow=3, green=4

  for (int i=0; i<4;++i) {
    lfo[i].wave=RAMP;
//    EEPROM.get(eeAddress++, lfo[i].wave); // byte variable
    lfo[i].rate1=1000000;  // initial LFO rates
    lfo[i].rate2=1000000;
//    lfo[i].amplitude=1023;
    lfo[i].minimum=0;
    lfo[i].maximum=0; // all lfos off
//    EEPROM.get(eeAddress, lfo[i].amplitude);
//    lfo[i].amplitude&=0x3ff; // in case there are bad values in eeprom
//    eeAddress+=2;  // int variable
  }

  for (int i=0; i<NPOTS;++i) sample_pot(i); // initialize pot settings
  lockpots(); // set high initial movement threshold
  mode=CCMODE;
  FlexiTimer2::set(1, lfo_generate); // set up 1ms call to DDS 
  FlexiTimer2::start(); 
}
  
void loop() {
  uint8_t reading;
  int i,j;

// handle CC page switching button
  if (!digitalRead(BUTTON1)) {
    page++;  // switch CC page
    lockpots();  // set higher movement threshold when CC page changes
    if (page > 4) {
      page=0;  // only 5 pages
      mode=CCMODE;
    }
    if (page == 4) {
      mode=LFOMODE;
    }  
    setleds(1<<page);
    while (!digitalRead(BUTTON1)) delay(10); // wait for button release
  }

 // handle MIDI channel selection button - only in CC mode
  if ((mode==CCMODE) && !digitalRead(BUTTON2)) {
    while (!digitalRead(BUTTON2)) {
      if ((millis()/LEDFLASH) &1) setleds(channel+1); // display as channels 1-16, 16 is all leds off 
      else setleds(0);
      delay(10); // wait for button release   
    }
    button2timer=millis(); 
    while ((millis()-button2timer) < BUTTON2_TIME) { // more button presses to increment channel
      if (!digitalRead(BUTTON2)) {    
        channel++;  // increment MIDI channel
        channel=channel&0xf;  // 16 channels 0 -15
        setleds(channel+1); // display as channels 1-16, 16 is all leds off
        while (!digitalRead(BUTTON2)) delay(10); // wait for button release   
        button2timer=millis();
      }
      if ((millis()/LEDFLASH) &1) setleds(channel+1); // display as channels 1-16, 16 is all leds off 
      else setleds(0);
    }
    setleds(1<<page); // set back to page number
  }
      
  if (mode==CCMODE ) { // send CC messages if pots change
    for (i=0;i<NPOTS;++i) {
      if (CC[i] != (reading=sample_pot(i)/8)) { // convert pot value 0-1023 to CC value 0-127
       CC[i]=reading;
        controlChange(channel,BASE_CC+i+page*16, CC[i]);
        MidiUSB.flush();      // forces message to be sent now
      }
    }
  }
  
  if (mode==LFOMODE) {    // page 4 is LFOs
    if (digitalRead(BUTTON2)==0) { // shift button is pressed
      if (((millis() - button2timer) > DEBOUNCE) && (shift==0)) {
        shift=1; // has been pressed long enough for debounce so flag button as active
        lockpots(); // pot values are locked till a significant change is made
      }
    }
    else { // shift button is not pressed
      if (shift) { // button was just released
        shift=0;
        lockpots(); // pot values are locked till a significant change is made
      }
      else button2timer=millis(); // button not pressed so reset the button timer
    }
 //  set rate, waveshape and amplitude parameters
      double t;
      for (i=0; i<4;++i) {
        sample_pot(i*4);
        if (potlock[i*4]==0) {
          t=double(1024-pot[i*4])/RANGE;
          lfo[i].rate1=(long)(pow(10,t))+MIN_DELTA; // exponential pot response for ramp rates for a very large time range
        }
        sample_pot(i*4+1);
        if (potlock[i*4+1]==0) {
          t=double(1024-pot[i*4+1])/RANGE;
          lfo[i].rate2=(long)(pow(10,t))+MIN_DELTA;     
        }
        sample_pot(i*4+2);
        if (potlock[i*4+2]==0) lfo[i].wave=pot[i*4+2]/WDIV;
        sample_pot(i*4+3);
        if (potlock[i*4+3]==0) {
          if (shift) lfo[i].minimum=pot[i*4+3]/8; 
          else lfo[i].maximum=pot[i*4+3]/8; 
        if (lfo[i].maximum < lfo[i].minimum) lfo[i].minimum=lfo[i].maximum; // handle cases of min pot > max pot
        if (lfo[i].minimum > lfo[i].maximum) lfo[i].maximum=lfo[i].minimum;
      }    
    }
    
    // update LFO outputs if they have changed
    for (i=0;i<4;++i) {
      unsigned int ccval=map(lfo[i].dacout,0,4097,lfo[i].minimum, lfo[i].maximum);
      if (lfo[i].lastCC != ccval) {  // don't send CC unless it has changed
        lfo[i].lastCC=ccval;
        if (lfo[i].maximum > 2) { // if max pot is near zero mute the lfo
          controlChange(channel,BASE_CC+i+page*16, ccval);
          MidiUSB.flush();      // forces message to be sent now
        }
      }
    }
    analogWrite(RED_PIN,lfo[0].lastCC * 2); // show LFO state on leds - scale CC up for analogWrite range 0-255
    analogWrite(ORANGE_PIN,lfo[1].lastCC * 2); 
    analogWrite(YELLOW_PIN,lfo[2].lastCC * 2); 
    analogWrite(GREEN_PIN,lfo[3].lastCC * 2); 
  }
}
