/* ------------------------------------------------------------------------ *
 * 
 * Modulo Two from 514A-4 to Arinc-429 for HFS-700 sketch                   
 *
 * Receive Modulo 2 on 2400 baud, continious stream of 32 bit words,
 *    Frequency and Mode select data transfer to Arinc,
 *    Squelch data to MCU pins resistor selector.
 *    Key, Vox and Tune bits are ignored.
 *    
 * Transmit Arinc words 12.5kbs, 32 bits.                                    
 * 
 * ------------------------------------------------------------------------ *                                                         
 *    
 *    The receive function implementation:
 *
 * The MCU has a build-in analog comparator, pins D6 and D7. This is used as
 * receiver for the data stream from the 514A panel.
 * The Analog Comparator interrupt, on a changing comparator output, toggles 
 * clockFlag and restarts Timer 2. An interrupt while clockFlag is set 
 * generates a data stream sample handling. A valid decode sets the mod2Event 
 * flag for further handling by the (main) loop().
 * 
 * Timer 2 overflow interrupt is used to set the clockFlag and 
 * take a data stream sample at the intended moment.
 * 
 *    The transmit implementation:
 *    
 * A separate function call is made to the transmit function, only when valid 
 * data have been detected.  
 * To avoid timing problems the Arinc timing is following the Modulo 2 stream.
 * Ten Modulo 2 words time lengths are more or less 135 ms, the required rate.
 * Bit timing during transmit is done with a microsecond delay function.
 * 
 * A counter is used here for MCU output pin relay control, Digital 3.
 * 
 * The RF-Sensitivity signal for the HFS-700 is constructed by switching 
 * a set of resistors by the MCU. Analog pins 0-6 and Digital 2 are used.
 * 
 *    The diagnostic signals:
 *    
 * Arduino standard LED pin, D13, indicates that the Module 2 receiver is in 
 * synch condition, providing valid data.
 * pinTrigger, D10, goes HIGH at first Arinc word, LOW at second and can be 
 * used to trigger an oscilloscope when analizing the Arinc A and B signals,
 * that are present on D12 and D11 pins.
 * D9 pin, pinSample, follows the decoded Modulo 2 stream.
 * D8 pin, pinSync, is set during sync-bit interval of the Modulo 2 stream.
 * 
 * Loop() contains routines to check validity of the frequency selection!
 * These are commented out since the HFS-700 transceiver has it's own check.
 *
 * NB. The Arinc signal will only be present while a valid Modulo two stream
 *     is received.
 *     
 * Synch LED changed to ERROR indication and Watchdog to signal loss of the 
 * Modulo Two stream
 *-------------------------------------------------------------------------- */

#include <Watchdog.h>

const byte    pinLed       =   13; // Arduino's build-in LED, here ERROR
const byte    pinArincA    =   12; // Arinc HIGH output pin
const byte    pinArincB    =   11; // Arinc LOW output pin
const byte    pinTrigger   =   10; // HIGH at first Arinc Word, LOW at second
const byte    pinSample    =    9; // decoded module 2 serial signal
const byte    pinSync      =    8; // for testing
const byte    pinRelay     =    3; // the "ON" delay relay
const byte    arincLabel   = 0xF8; // reverse order of label 037, MSB first
const byte    onDelayCount =   16; // 16 times Arinc period of 135 ms

// used in isr's
volatile unsigned long      mod2Word;     // samples result
volatile unsigned long      com2Word;     // previous result to compare
volatile unsigned long      xor2Word;     // to check for sync bit
volatile boolean            clockFlag;    // keep track of clock events
volatile boolean            mod2Event;    // flag to start recoding
volatile boolean            mod2Synch;    // flag indicating synchronised
volatile boolean            sample;       // received bit of word
volatile byte               bitCount;     // keep track of received bits
volatile byte               state = 0;    // keep track of decoder states
// Arinc stuf
volatile unsigned long      lbl037A; // ARINC429 word 1 word ID 1
volatile unsigned long      lbl037B; // ARINC429 word 2 word ID 1
volatile unsigned long      lbl037C; // ARINC429 word 1 word ID 2
volatile unsigned long      lbl037D; // ARINC429 word 2 word ID 2
volatile byte               WordSelect = 0;  // Arinc word to send
volatile byte               ariCount = onDelayCount; // timing for on-delay

Watchdog watchdog; // used to check for presence of a valid Module 2 stream

void setup() 
{
  // Initialize ARINC HIGH/LOW OUT pins as outputs
  pinMode(pinLed, OUTPUT);            // build-in LED
  pinMode(pinArincA, OUTPUT);         // ARINC HIGH
  pinMode(pinArincB, OUTPUT);         // ARINC LOW
  pinMode(pinTrigger, OUTPUT);        // ARINC word trigger pin
  pinMode(pinSample, OUTPUT);         // sample pin
  pinMode(pinSync, OUTPUT);           // test pin
  pinMode(pinRelay, OUTPUT);          // on - off relay

  // pin 6 and 7, AIN0 and AIN1 are in use as comparator inputs
  // pin 4 and 5 are not used, pin 0 and 1 are RX and TX during boot- up
  pinMode(A0, OUTPUT);                // RF SENSITIVITY control
  pinMode(A1, OUTPUT);                // RF SENSITIVITY control
  pinMode(A2, OUTPUT);                // RF SENSITIVITY control
  pinMode(A3, OUTPUT);                // RF SENSITIVITY control
  pinMode(A4, OUTPUT);                // RF SENSITIVITY control
  pinMode(A5, OUTPUT);                // RF SENSITIVITY control
  pinMode( 2, OUTPUT);                // RF SENSITIVITY control
  
  digitalWrite(pinLed, HIGH); // ERROR indicator
  
  // setup data
  lbl037A = 0x00000000;  // initial data
  lbl037B = 0x00000000;  // initial data
  lbl037C = 0x00000000;  // initial data
  lbl037D = 0x00000000;  // initial data
      
  lbl037A |= arincLabel; // embed label in word 1
  lbl037B |= arincLabel; // embed label in word 2
  lbl037C |= arincLabel; // embed label in word 3
  lbl037D |= arincLabel; // embed label in word 4

  // Setup the Watchdog timer

  watchdog.enable(Watchdog::TIMEOUT_2S); // prepare the watchdog

  // Setup analog comparator interrupt
  ADCSRA = (ADCSRA & 0b01111111) | 0b00000000; // bit ADEN off
  ADCSRB = (ADCSRB & 0b10111111) | 0b00000000; // bit ACME off
  ACSR = (ACSR & 0b00110000) | 0b00001000; // interrupt on comparator toggle
  // power save
  DIDR1 |= ((1<<AIN1D) | (1<<AIN0D)); // disable digital buffer
  PRR = 0b10111111; // only timer2 is powered

  // Setup timer 2 interrupt
  PRR = PRR & 0b10111111; // to be sure timer 2 is powered
  TCCR2A = (TCCR2A & 0b00110000) | 0b00000000; // force normal mode
  TCCR2B = (TCCR2B & 0b11110000) | 0b00000011; // prescaler clock /32
  TIMSK2 = (TIMSK2 & 0b11111111) | 0b00000001; // timer overflow interrupt
      
  interrupts(); // enable interrupts
}
/*-------------------------------------------------------------------------- */

// Arinc word transmit function

void transmitWord() {
   
  unsigned long txreg; // transmit register

  if (WordSelect == 1) {
    txreg = lbl037C;
    WordSelect = 0; // next word
    digitalWrite(pinTrigger, LOW);
  }
  else {
    txreg = lbl037A;
    WordSelect = 1; // next word
    digitalWrite(pinTrigger, HIGH);
  }
  // on - off relay delay timing handler
  if (ariCount > 0) // this only runs for short time after mode change to "ON"
    {
      digitalWrite(pinRelay, LOW); // relay off
      ariCount--;
      if (ariCount == 0)
      { 
        digitalWrite(pinRelay, HIGH); // yeah, on!
      }
    }
  
  for (byte ptr = 0; ptr < 32; ptr++) { // 32 bits to go
    if (txreg & 1) {                // check bit status
      // transmit '1'
      digitalWrite(pinArincA, HIGH);
      delayMicroseconds(36);        // pause, adapt to get 12.5 kb/s
      digitalWrite(pinArincA, LOW);
      delayMicroseconds(36);        // pause
    }
    else {
      // transmit '0'
      digitalWrite(pinArincB, HIGH);
      delayMicroseconds(36);        // pause
      digitalWrite(pinArincB, LOW);
      delayMicroseconds(36);        // pause
    }
      txreg = txreg >> 1;           // update transmit register txreg
  }
  bitCount = bitCount + 5; // bridging the gap.. 6 is too much
  state = 0; // last mod2 capture is invalid due this interrupt
}

// interrupt service routines

ISR(ANALOG_COMP_vect) { 
  // analog comparator interrupt on every comparator toggle
  int number_of_set_bits;

  TCNT2 = 100; // 75; // setup timer / counter
  if (clockFlag) {
    digitalWrite(pinSample, sample); // for testing
    digitalWrite(pinSync, LOW);
    bitWrite(mod2Word, bitCount, sample);  // write received bit to mod2Word
    bitCount = bitCount + 1; // next bit
    if (bitCount >= 32) { // another word is complete
      bitCount = 0; // start next word capture
      switch (state) {
        case 0: // first word read, save
          state = 1; // next
          com2Word = mod2Word; // save last received word for next compare
        break;
        case 1: // second word read, check syncbit or shift to synchronize
          xor2Word = mod2Word ^ com2Word;
          number_of_set_bits = __builtin_popcountll(xor2Word); // check "1" bits
          if (number_of_set_bits == 1) {
            if (xor2Word == 0x80000000) { // syncbit is found
              digitalWrite(pinSync, HIGH);
              mod2Event = 1; // com2Word is valid result, process
              mod2Synch = 1; // indicate synched
              com2Word = mod2Word; // save last received word for next compare
            } else { // out of sync
              mod2Synch = 0; // sync lost
              bitCount ++; // increment to synchronize
              state = 0; // return to get shifted words
            }
          } else {
            state = 0; // invalid result
          }
        break;
      }
    }
  }
  clockFlag = !clockFlag;
}

ISR(TIMER2_OVF_vect) {
  // timer2 overflow handler
  clockFlag = 1; // force clockEvent at next pinchange
  sample = bitRead(ACSR, ACO); // read comparator output
}

/* -------------------------------- *
 * Parity processing                *
 * Standard label037B is processed  *
 * Input : none                     *
 * Output: parity status            *
 * -------------------------------- */
byte parityB(void) {

  byte prt[4];
  prt[3] = (lbl037B >> 24) &0x7F;
  prt[2] = (lbl037B >> 16) &0xFF;
  prt[1] = (lbl037B >> 8)  &0xFF;
  prt[0] = lbl037B &0xFF;
  prt[0] = prt[0] ^ prt[1] ^ prt[2] ^ prt[3];
  prt[0] = prt[0] ^ (prt[0] >> 4);
  prt[0] ^= (prt[0] >> 2);
  prt[0] ^= (prt[0] >> 1);
  prt[0] &= 1;
  return prt[0];
}

/* -------------------------------- *
 * Parity processing                *
 * Standard label037D is processed  *
 * Input : none                     *
 * Output: parity status            *
 * -------------------------------- */
byte parityD(void) {

  byte prt[4];
  prt[3] = (lbl037D >> 24) &0x7F;
  prt[2] = (lbl037D >> 16) &0xFF;
  prt[1] = (lbl037D >> 8)  &0xFF;
  prt[0] = lbl037D &0xFF;
  prt[0] = prt[0] ^ prt[1] ^ prt[2] ^ prt[3];
  prt[0] = prt[0] ^ (prt[0] >> 4);
  prt[0] ^= (prt[0] >> 2);
  prt[0] ^= (prt[0] >> 1);
  prt[0] &= 1;
  return prt[0];
}

/*---------------------------------------------------------------------------*/

void loop() {

  boolean     key; // signal retrieved from mod2
  boolean   voice; // signal retrieved from mod2
  boolean    tune; // signal indicating frequency change
  int     squelch; // 3 bit value squelch knob
  static int      oldersq; // previous value
  static int    arincTime; // counter to keep track of timing
  byte    modeSel; // 3 bit value mode selector
  static unsigned long   hackWord; // word copy used in processing

  digitalWrite(pinLed, !mod2Synch); // indicate error when sync lost

  if (mod2Event) { // wait... runs only on mod2Event
    watchdog.reset(); // by valid module 2 stream
    mod2Event = 0; // clear flag
    arincTime ++; // increment
    if (hackWord != com2Word) { // 
      hackWord = com2Word; // save result for further handling
      // prepare data update
      (hackWord & 0x00000001)? key = 1 : key = 0;
      (hackWord & 0x00000002)? voice = 1 : voice = 0;
      (hackWord & 0x40000000)? tune = 1 : tune = 0;
      squelch = ((hackWord >>2) & 0x00000007); // read squelch info
      modeSel = ((hackWord >>5) & 0x00000007); // read mode selector info
  
      // recode frequency data 
      lbl037B = (lbl037B & 0xE00007FF) | ((hackWord >>  1) & 0x1FFFF800);
      lbl037D = (lbl037D & 0xE00007FF) | ((hackWord << 17) & 0x1E000000);
      lbl037D |= 0x00000100; // set ident bit 9 to 1
  
      // recode mode 
      // word bit 10: 0=AM, 1=SSB and word bit 11: 0=LSB, 1=USB
      switch(modeSel) {
        case 0: // OFF
        lbl037B = (lbl037B &= 0xFFFFF9FF);
        ariCount = 16; // load the on delay timer routine
        break;
        case 1: // USB
        lbl037B = (lbl037B & 0xFFFFF9FF) | 0x00000600;
        break;
        case 2: // LSB
        lbl037B = (lbl037B & 0xFFFFF9FF) | 0x00000200;
        break;
        case 3: // AM
        lbl037B = (lbl037B & 0xFFFFF9FF) | 0x00000400;
        break;
        default: // other, treat like AM
        lbl037B = (lbl037B & 0xFFFFF9FF) | 0x00000400;
        break;
      }
  
/*      if ((lbl037B & 0x1FFFF800) > 0x11CCC800) {
        // recover to upper limit 23.999 MHz and set SSM to NCD of word ID 1
        lbl037B = (lbl037B & 0x900007FF) | 0x31CCC800;
        lbl037D = (lbl037D & 0x9E0001FF) | 0x60000100;  // set SSM to NCD
        goto skipresetssm;
      } 
      if ((lbl037B & 0x1FFFF800) < 0x01400000) {
        // recover to lower limit 2.800 MHz and set SSM to NCD of word ID 1
        lbl037B = (lbl037B & 0x900007FF) | 0x21400000;
        lbl037D = (lbl037D & 0x9E0001FF) | 0x60000100;  // set SSM to NCD
        goto skipresetssm;
      }*/
      if (modeSel >= 4) { // all other mode selections are TEST
        lbl037B = lbl037B & 0x9FFFFFFF | 0x40000000; // set SSM to TEST
        goto skipresetssm; 
      }
          
      lbl037B = lbl037B & 0x9FFFFFFF; // set SSM to NORMAL of word ID 1
      lbl037D = lbl037D & 0x9FFFFFFF; // set SSM to NORMAL of word ID 2
          
      skipresetssm:
      /* proces parity */ 
      parityB()? lbl037B &= 0x7FFFFFFF : lbl037B |= 0x80000000; // bit 32
      parityD()? lbl037D &= 0x7FFFFFFF : lbl037D |= 0x80000000; // bit 32

      // RF SENSITIVITY variable resistance control
     if (squelch != oldersq) {
        digitalWrite(2, LOW);
        PORTC = 0; // all resistors off
        switch(squelch) {
          case 1:
          digitalWrite(A0, HIGH); // switch resistor on
          break;
          case 2:
          digitalWrite(A1, HIGH);
          break;
          case 3:
          digitalWrite(A2, HIGH);
          break;
          case 4:
          digitalWrite(A3, HIGH);
          break;
          case 5:
          digitalWrite(A4, HIGH);
          break;
          case 6:
          digitalWrite(A5, HIGH);
          break;
          case 7:
          digitalWrite(2, HIGH);
          break;
        }
        oldersq = squelch; // update previous value
      }
    }
    // call the Arinc word transmit function
    if (arincTime >= 7) {
      arincTime = 0; // reset
      noInterrupts(); // stop mod2 decoder
      lbl037A = lbl037B; // data exchange
      lbl037C = lbl037D; 
      transmitWord(); // send an Arinc word
      interrupts(); // restart mod 2 decoder
    }
  }
}
//--------------------------------- -o-O-o- ---------------------------------
