/*
  Simple Lathe Controller based on Arduino Nano and HCTL2000
  
  Author:
  Log:  HdW, Aug 2018 - Draft 1, Tested
  ToDo:   
            
   Limits calculations:
   If the HCTL clock input is 14 MHZ, accordning to the datasheet, the maximum input pulse frequency is 14/3 MHZ or 4,67MHZ
   this equates to 214.3 nS per pulse. 
   If the 8 bit mode (SEL High) is used we can count 127 pulses before counter overflow or 127 * 214 nS = 27.2 uS 
   
   The Arduino port read speed should be faster than 36.7 KHz or at least once every 27.2 uS, using digitalWrite() to 
   select OE will take approximately 8uS. Then for 3 counters this will be 24uS, leaving little margin for other tasks.
    
   For the 12 bit mode, 2048 * 214 nS = 438.9 uS. Here we will also need to toggle the select line for each counter adding 
   another 26uS with the standard digitalWrite. For all three counters OE and SEL in total 52uS. This leaves around 380uS 
   for other tasks. I would like to keep the extra line open for Lathe and VFD Controlls...

   Using direct port writes and reads with ~125ns, per counter would take 500ns for all three 1.5uS
   OE1 = 0;   // 125ns
   read byte D0-5; //125 ns
   read byte D6-7; //125ns
   OE1 = 1;  // 125ns

   conclusion; using direct port reads/writes with the 8 bit mode should leave enough processing time for serial communication,
   calulations and enough free pins for lathe and VFD controlling. 
 
   Feeds are very slow, so no problem there.
   For large lead angle threads like 1 TPI, the carriage would need to travel 25,4mm per rpm
   For a 2000 rpm spindle speed, which is unusualy high for threading, the spindle will rotate 33 times per second, 
   or 838.2mm. Then 838.2/0,005 = 167640 Pulses per second, or 5.9uS per pulse; 
   this is aproximately 30 times slower than the maximum input frequency for the counters.

   Input switches are scanned with oe1-oe3 and swA-swD 
   swA - SwD are mapped to PORTC and have internal pullups resistors enabled. When the input is read on  port C from 
   the external counter the switch status is also read and updated.
   
   PC2 -[]- oe1 

Updates:
 First measurements show read pulses of 125ns
 The read routine is fast enough, but using Arduino Serial.write takes aproximately 650uS per cycle. Will have to convert
 this to an interup driven routine with circular buffer.
 If the spindle runs at 3000 RPM or 50HZ, with a 1000CPR Encoder this will be 50KHz , or 50e3 PPS, still well in limits 
 of 8MHz counter clock.
 
 
For reference, and code snips, ideas, credits are due:

 http://skpang.co.uk/blog/archives/323
 https://efundies.com/avr-bitwise-operations-in-c/
 https://inst.eecs.berkeley.edu/~ee128/fa04/labs/hctl2016.pdf
 http://www.farnell.com/datasheets/312480.pdf
 https://www.torretje.nl/files/p8255optim.ino
 https://arduino.stackexchange.com/questions/16698/arduino-constant-clock-output
 https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
*/
 


#define d2 2 // PD2  14
#define d3 3 // PD3  13
#define d4 4 // PD4  12
#define d5 5 // PD5  11
#define d6 6 // PD6  10
#define d7 7 // PD7  9

#define d0 14 // PC0 1
#define d1 15 // PC1 15

#define swA 16 // PC2
#define swB 17 // PC3
#define swC 18 // PC4 
#define swD 19 // PC5

#define res 8 //PB0 5
#define clk 9 // PB1 2

#define oe1 10 //PB2 4
#define oe2 11 //PB3
#define oe3 12 //PB4
#define xxx 13 //PB5

#define ENC_A 16
#define ENC_B 17
#define ENC_PORT PINC

static int8_t count[3] = {0x0,0x0,0x0}; 
static int8_t count_old[3] = {0x0,0x0,0x0}; 
static uint8_t buttons[3] = {0x0,0x0,0x0}; 
static long  count_position[3] = {0x0,0x0,0x0};

void setup() {     

   pinMode (d0, INPUT); 
   pinMode (d1, INPUT); 
   pinMode (d2, INPUT); 
   pinMode (d3, INPUT); 
   pinMode (d4, INPUT); 
   pinMode (d5, INPUT); 
   pinMode (d6, INPUT); 
   pinMode (d7, INPUT); 
   pinMode (res, OUTPUT); 
   pinMode (clk, OUTPUT); 
   pinMode (oe1, OUTPUT); 

   PORTC =  0b00111100; // pullups for button inputs
  
  digitalWrite(res,LOW);
  digitalWrite(res,HIGH);

  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  
  // set up 8 MHz timer on CLOCKOUT (OC1A)
  pinMode (clk, OUTPUT); 
  // set up Timer 1
  TCCR1A = bit (COM1A0);  // toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);   // CTC, no prescaling
  OCR1A =  0;       // output every cycle
  
  // 
  Serial.begin (115200);
  Serial.println("Start");

}


void readCounters(){
  // enable counter output
   
  PORTB &= ~_BV(2); // Set bit 2 low
//  count[0] = (PIND & 0b11111100) | ( PINC & 0b00000011);
  count[0] = PIND & 0b11111100 
  count[0] |= PINC & 0b00000011;
  buttons[0] = PINC & 0b00111100;
  PORTB |= _BV(2); // Set bit 2 high
  //PORTB &= ~_BV(0); // reset low
  //PORTB |= _BV(0);  // reset high
  
   // enable counter output
 
  PORTB &= ~(1<<3); // Set bit 3 low
  count[1] = (PIND & 0b11111100) | ( PINC & 0b00000011);
  PORTB |= 1<<3; // Set bit 3 high
  //PORTB &= ~_BV(0); // reset low
  //PORTB |= _BV(0);  // reset high
  
   // enable counter output

  PORTB &= ~(1<<4); // Set bit 4 low
  count[2] = (PIND & 0b11111100) | ( PINC & 0b00000011);
  PORTB |= 1<<4; // Set bit 4 high
  PORTB &= ~_BV(0); // reset low
  PORTB |= _BV(0);  // reset high
}

// 
    // should be read every 127 encoder pulses
    // 2048 bits if SEL is used in 12 bit mode 
void updatePosition() {
  
  int delta = 0;
  
  for ( int i = 0; i < 3; i++ ) {
    // if the current count is less the old count are more than 127 bit apart
    
    // then counter overrun error
    // else update position
    
    delta = count[i] - count_old[i]; // get the difference since last run
    //count_old[i] = count[i];
   
   // if ( delta > 127 ){  count[i] = count[i] - 255; } // counter overflow assume one cycle
   // if ( delta < -127){  count[i] = count[i] + 255;  } // counter overflow assume one cycle
    // Serial.println(delta, DEC);
    count_position[i] +=  delta; 
    
  }
   //Serial.println(count_position[0], DEC);
}

 /* returns change in encoder state (-1,0,1) */
int8_t readEncoder()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

void updateRotEnc(){
  static uint8_t counter = 0;      //this variable will be changed by encoder input
 int8_t tmpdata;
 /**/
  tmpdata = readEncoder();
  if( tmpdata ) {
    Serial.print("Counter value: ");
    Serial.println(counter, DEC);
    counter += tmpdata;
  }
}
void debug() {
   //int var = (PORTD & B11111100) | ( PORTC & B00000011);
   DDRC |= 0x00; 
  int var =  PINC & 0b00000001;
 // var = PORTC & _BV(0)
if (Serial.available() > 0) {
                // read the incoming byte:
               // incomingByte = Serial.read();

                // say what you got:
                Serial.print("in> : ");
                Serial.println(var, HEX);
                pinMode (A0, INPUT);
                Serial.println(digitalRead(A0),HEX);
        }
}
void loop() {
   //  debug();
    readCounters();
    updatePosition();
        
    Serial.print(F("X"));
    Serial.print((long)count_position[0]);
    Serial.println(F(";"));

   // Serial.print(F("Z"));
   // Serial.print((long)position[1]);
   // Serial.println(F(";"));

   // Serial.print(F("T"));
   // Serial.print((unsigned long)position[2]);//tachReadoutMicrosec
   // Serial.print(F("/"));
   // Serial.print((unsigned long)position[2]); // tach position
   // Serial.println(F(";"));
 
  
}
