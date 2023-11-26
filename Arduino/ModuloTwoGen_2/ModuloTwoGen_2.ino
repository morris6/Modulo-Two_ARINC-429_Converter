/*
 * Modulo two generator
 * 
 * Generates a Modulo two signal on digital pin number x.
 * 
 * Uses timer 2. Digital pin 10 is occupied.
 * 
 * Changes frequency and squelch data every 2 seconds
 */
// constants
const int pinOC2A = 10; // timer2 compare A signal
const int pinOut = 9; // our generated signal
const int pinTest = 2; // for testing
const int pinFreq = 3; // frequency change toggle
// variables
byte f10MHz = 0;
byte f1MHz = 3;
byte f100kHz = 7;
byte f10kHz = 8;
byte f1kHz = 5;
byte f100Hz = 0xF;
byte mode = 2; // off-0, usb-1,  lsb-2, am-3
byte squelch = 7;
byte keyVox = 0;
// used in interrupt routine
volatile unsigned long signalWord = 0x0378505C;
volatile boolean syncFlag;
volatile int bitCount;
volatile boolean phase;
volatile boolean nextWord;

volatile int countWord;
volatile boolean freqToggle = 0;


void changeFrequency(void) {
  signalWord = 0; // start with all zeroes
  signalWord += (f10MHz & B00000011); // load data, tune bit
  signalWord = signalWord << 4; // shift up to make space
  signalWord += (f1MHz & B00001111);
  signalWord = signalWord << 4; // 12 bits there
  signalWord += (f100kHz & B00001111);
  signalWord = signalWord << 4; // 16 bits
  signalWord += (f10kHz & B00001111); // load data
  signalWord = signalWord << 4; // 20
  signalWord += (f1kHz & B00001111);
  signalWord = signalWord << 4; // 24 bits there
  signalWord += (f100Hz & B00001111);
  signalWord = signalWord << 3; // 27 bits
  signalWord += (mode & B00000111);
  signalWord = signalWord << 3; // 30 bits
  signalWord += (squelch & B00000111);
  signalWord = signalWord << 2; // 32 bits
  signalWord += (keyVox & B00000011); // complete
  digitalWrite(pinFreq, freqToggle);
  freqToggle = !freqToggle;
}

void setup() {
  // put your setup code here, to run once:
  // compose a signalWord
  signalWord = 0; // start with all zeroes
  signalWord += (f10MHz & B00000011); // load data, tune bit
  signalWord = signalWord << 4; // shift up to make space
  signalWord += (f1MHz & B00001111);
  signalWord = signalWord << 4; // 12 bits there
  signalWord += (f100kHz & B00001111);
  signalWord = signalWord << 4; // 16 bits
  signalWord += (f10kHz & B00001111); // load data
  signalWord = signalWord << 4; // 20
  signalWord += (f1kHz & B00001111);
  signalWord = signalWord << 4; // 24 bits there
  signalWord += (f100Hz & B00001111);
  signalWord = signalWord << 3; // 27 bits
  signalWord += (mode & B00000111);
  signalWord = signalWord << 3; // 30 bits
  signalWord += (squelch & B00000111);
  signalWord = signalWord << 2; // 32 bits
  signalWord += (keyVox & B00000011); // complete
  
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinOC2A, OUTPUT); // set OC2A as output
  pinMode(pinOut, OUTPUT); // the signal output
  pinMode(pinTest, OUTPUT); // the test output
  pinMode(pinFreq, OUTPUT); // the freq toggle
  // setup timer 2
  PRR0 = PRR0 & B10111111; // to be sure timer 2 is working
  TCCR2A = (TCCR2A & B00111100) | B01000010; // toggle on compare, clear timer on compare
  TCCR2B = (TCCR2B & B11110000) | B00000011; // prescaler clock /32
  OCR2A = 103; // resulting frequency about 2400 Hz ? actually 2404.8 Hz
  
  // setup interrupt
  TIMSK2 = (TIMSK2 & B11111111) | B00000010;
  interrupts(); // enable interrupts
}

ISR(TIMER2_COMPA_vect) {
  digitalWrite(pinTest, syncFlag); // test
  phase = digitalRead(pinOC2A);

      countWord ++;
      if (countWord >= 9600) {
        countWord = 0;
        f1kHz ++;
        if (f1kHz == 10) {
          f1kHz = 0;
          f10kHz ++;
          if (f10kHz == 10) {
            f10kHz = 0;
          }
        }
        mode ++;
        if (mode >= 4) {
          mode = 1;
        }
        squelch ++;
        if (squelch >= 8) {
          squelch = 0;
          mode = 0; // this shifts off mode
        }
        changeFrequency();
      }

  
  if (!phase) { // Modulo Two clock falling
    if (bitCount == 31) {
      digitalWrite(pinOut, (phase ^ syncFlag));
    } else {
      digitalWrite(pinOut, (phase ^ bitRead(signalWord, bitCount)));
    }
  } else {
    if (bitCount == 31) {
      digitalWrite(pinOut, (phase ^ syncFlag));
      syncFlag = !syncFlag; // toggle synch bit
      bitCount = 0; // reset
    } else {
      digitalWrite(pinOut, (phase ^ bitRead(signalWord, bitCount)));
      bitCount = bitCount + 1;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (nextWord) {
    nextWord = 0;
  }
}
