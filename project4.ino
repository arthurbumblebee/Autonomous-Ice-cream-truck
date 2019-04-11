/*
 * Arthur Makumbi  
 * CS342 Project 4 
 * Ice cream truck
*/

#include <avr/io.h>     // for register names
#include "USART.h"      // for comm over USB

// Infrared communication libraries
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLibRecv.h>

// The Mini IR Remote's button codes
#define BUTTON_RIGHT  0xfd50af 
#define BUTTON_LEFT   0xfd10ef 
#define BUTTON_SELECT 0xfd906f 
#define BUTTON_UP     0xfda05f 
#define BUTTON_DOWN   0xfdb04f
#define BUTTON_0      0xfd30cf
#define BUTTON_1      0xfd08f7
#define BUTTON_2      0xfd8877
#define BUTTON_3      0xfd48b7
#define BUTTON_4      0xfd28d7
#define BUTTON_5      0xfda857
#define BUTTON_6      0xfd6897
#define BUTTON_7      0xfd18e7
#define BUTTON_8      0xfd9867
#define BUTTON_9      0xfd58a7

IRdecodeNEC nec;          // an NEC decoder object
IRrecv rx( 2 );           // the TSOP38238 IR receiver is connected to pin 2
uint32_t valuePrevious;   // handles NEC repeat codes

/* motor motion variables */
bool start = false;
bool playMusic = false;
const int N = 10;
uint16_t sensedADC = 0;
float meanADC = 0.0;

/* music variables */
const int numberOfNotes = 15;

//frequencies of the musical notes for in the jungle, the lion sleeps tonight
float frequency[numberOfNotes] = {21.83, 24.50, 27.50, 24.50, 27.50, 
  30.87, 27.50, 24.50, 21.83, 24.50, 27.50, 24.50, 21.83, 27.50, 24.50 };

//int duration[numberOfNotes] = { 1500, 500, 250, 500, 500, 250, 500, 250, 
//500, 500, 250, 500, 500, 500, 250 };

// double the durations
int duration[numberOfNotes] = { 3000, 1000, 500, 1000, 1000, 500, 1000, 500, 
1000, 1000, 500, 1000, 1000, 1000, 500 };

int octave = 6;
int note = 0; // track the current index into frequency and duration
int noteIndex = 0;
int ms = 0;   // counter

char s[32];
char msg[128];

int motorSpeed = 60;
 
// required for timer1 comp b interrupt to play the song
ISR( TIMER1_COMPB_vect ){
  if(playMusic){
    ms++;
    note = noteIndex % numberOfNotes;
    if(0 <= note <= numberOfNotes-1){
      if (ms>= duration[note]){
        noteIndex++;
        ms = 0;
        sprintf(s, "y=%d \n", note);
//        printString(s);
        if(0 <= note <= numberOfNotes-1){
          float freq = frequency[note] * pow( 2, octave + 1);
          int period = (16000000 / freq) - 1 ;
          OCR1AH = (period & 0xFF00) >> 8;
          OCR1AL = (period & 0x00FF);
        }
      }
    }
  }
}

// required for timer0 comp a interrupt for ADC
ISR( TIMER0_COMPA_vect ){}

// executes when int1 is triggered to start or play
ISR( INT1_vect ){
  start = !start; // toggle between start and stop
}

// ADC interrupt logic
ISR( ADC_vect ){
  sensedADC = ADCL;
  sensedADC |= (ADCH & 0x03) << 8; 
  meanADC = meanADC*float(N-1) / float(N) + float(sensedADC)/ float(N);
}

// main method
int main() {

  /* Configure GPIO especially for the motors: */
  DDRD = 0b11111000; 
  DDRC = 0b00001100; 
  DDRB = 0b00111110; 

  /*configure direction*/
  PORTB |= 0b00010000; //set PB 4 to high
  PORTC = 0b00000100; // set PC2 to high
  PORTD = 0b10000000;

  /* External Interrupt Configuration :  */
  EICRA = 0b00001000; // trigger INT1 interrupts on falling edges
  EIMSK = 0b00000010; // INT1's ISR enabled

  /*timer 0 drives both left and right motor oc0a and oc0b*/
  TCCR0A = 0b10100011; //set timer 0 to non-inverting PWM
  TCCR0B = 0b00000101; //set timer 0 prescaler to 1024

  /* sound generation and control */
  TCCR1A = 0b00000000;  // CTC mode for timer 1 toggle pin pb1 disabled enabled on button press, compa toggles
  TCCR1B = 0b00001001;  // CTC mode for timer 1 and no prescaling (by 1)
  OCR1AH = (30577 & 0xFF00) >> 8; 
  OCR1AL = (30577 & 0x00FF);
  OCR1BH = (1600 & 0xFF00) >> 8; 
  OCR1BL = (1600 & 0x00FF);


    /* ADC Configuration :  collect periodic distance readings on pad A0 */
  ADMUX  = 0x41;  // right-adjusted ADC on pad A1 (PC3) w/ reference to 5V (Vcc)
  ADCSRA = 0xFF;  // ADC enabled & set up for automatic conversions and prescale = 128
  ADCSRB = 0x03;  // when enabled, ADC conversions will be auto-triggered by TIMER0_COMPA events
  DIDR0  = 0x08;  // disable unnecessary digital inputs to save power

    /* Configure serial bus to print debugging info */
  initUSART();
  printString("RESET\n");

  TIMSK0 = 0b00000010; // enable the timer
  SREG |= 0x80; //enable global interrupts

  rx.enableIRIn();  // start the receiver
  while (true) {
    // if an obstacle is too close: about 20cm, stop or if not start
    if(meanADC >= 590 || !start){ 
      OCR0A = 0;         
      OCR0B = 0;
    }
    if(playMusic){
      TCCR1A = 0b01000000; // enable pin toggle
      TIMSK1 = 0b00000100; // enable the timer
    }
    else{
      TCCR1A = 0; // disable pin toggle
      TIMSK1 = 0; // disable the timer
    }
    sprintf( msg, "sensedADC :\t%d\t meanADC :\t%d \n", sensedADC, (int) meanADC );
//    printString(msg);
      
    if( rx.getResults() ) {             // wait for a button press
       printByte( nec.decode() );    // decode the pulse train 
       printString( "\t0x" );
       printHexByte( (nec.value & 0xFF000000) >> 24 );
       printHexByte( (nec.value & 0x00FF0000) >> 16 );
       printHexByte( (nec.value & 0x0000FF00) >> 8 );
       printHexByte(  nec.value & 0x000000FF );
       printString( "\t" );
       if( nec.value == 0xFFFFFFFF )    // check to see if it's a repeat code
       { 
          nec.value = valuePrevious;    // if it's a repeat code, keep the previous value
       }
       switch( nec.value ) {            // respond to the button press: choose a behavior based on the value!
          case BUTTON_UP:     // move forward
          {
             OCR0A = motorSpeed;         
             OCR0B = motorSpeed;        
//             printString("U forward \n"); 
             break;
          }
          case BUTTON_LEFT:   // move left
          {
            OCR0A = motorSpeed;
            OCR0B = 0;
//            printString("L left\n"); 
            break;
          }
          case BUTTON_RIGHT:  // move right
          {
            OCR0A = 0;
            OCR0B = motorSpeed;
//            printString("R right\n"); 
            break;
          }
          case BUTTON_SELECT: // start the car
          {
            start = true;
//            printString("S power on \n"); 
            break;
          }
          case BUTTON_DOWN:   // stop
          {
            OCR0A = 0;         
            OCR0B = 0;
//            printString("D stop\n"); 
            break;
          }
          case BUTTON_0:      // stop the music
          {
            start = false;
            playMusic = false;
//            printString("0 power off\n");
            break;
          }
//          case BUTTON_1:      printString("1\n"); break;
//          case BUTTON_2:      printString("2\n"); break;
//          case BUTTON_3:      printString("3\n"); break;
//          case BUTTON_4:      printString("4\n"); break;
          case BUTTON_5: 
          {
            playMusic = true;
//            printString("5 play music\n");
            break;
          }
//          case BUTTON_6:      printString("6\n"); break;
//          case BUTTON_7:      printString("7\n"); break;
          case BUTTON_8:      
          {
            playMusic = false;
//            printString("8 stop music\n");
            break;
          }
//          case BUTTON_9:      printString("9\n"); break;
          default:            
          {
//            printString("?\n"); 
            break;
          }
       }
       valuePrevious = nec.value;
       rx.enableIRIn();
    }
  }  

  return 0;
}
