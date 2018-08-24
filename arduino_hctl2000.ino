/*
  Simple Lathe Controller based on Arduino Nano and HCTL2000
  
  Author:
  Log: HdW, Aug 2018 - Draft 1, not tested
  
  Limits calculations:
   If the HCTL clock input is 14 MHZ, the maximum allowed input pulse frequency is 14/3 MHZ or 4,67MHZ
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
  

For reference, and code snips, ideas, credits are due:

 http://skpang.co.uk/blog/archives/323
 https://efundies.com/avr-bitwise-operations-in-c/
 https://inst.eecs.berkeley.edu/~ee128/fa04/labs/hctl2016.pdf
 https://www.torretje.nl/files/p8255optim.ino
 https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino/
*/
 
#define ENC_A 14
#define ENC_B 15
#define ENC_PORT PINC

#define d2 2 // PD2
#define d3 3 // PD3
#define d4 4 // PD4
#define d5 5 // PD5
#define d6 6 // PD6
#define d7 7 // PD7

#define d0 8 // PB0
#define d1 9 // PB1

#define oe1 10 //PB2
#define oe2 11 //PB3
#define oe3 12 //PB4
#define res 13 //PB5


static uint8_t count[3] = {0x0,0x0,0x0}; 
static uint8_t count_old[3] = {0x0,0x0,0x0}; 
static long  position[3] = {0x0,0x0,0x0};

void setup() {     

  // initialize PORTD as input.
   //arduino d2 d3 d4 d5 d6 d7 <- d0 - d5
  DDRD = DDRD | B00000000;

  //arduino d8, d9, d10 d11 d12 <- d6 d7, ->cs1 cs2 cs3
  DDRB = DDRB | B00011100;

  digitalWrite(res,HIGH);

  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);

  // 
  Serial.begin (115200);
  Serial.println("Start");

}


void readCounters(){
  // enable counter output
  // digitalWrite(oe1,LOW);       // 4.55 uS
  PORTB |= 1<<2; // Set bit 2 high
    // read PORTD and PORTB
  count[0] = (PORTD & B11111100) | ( PORTB & B00000011);
  //digitalWrite(oe1,HIGH);      // 3.95 uS
  PORTB &= ~(1<<2); // Set bit 2 low
  
   // enable counter output
  //digitalWrite(oe2,LOW);      // 4.55 uS
  PORTB |= 1<<3; // Set bit 3 high
    // read PORTD and PORTB
  count[1] = (PORTD & B11111100) | ( PORTB & B00000011);
  //digitalWrite(oe2,HIGH);     // 3.95 uS
  PORTB &= ~(1<<3); // Set bit 3 low
  
   // enable counter output
  // digitalWrite(oe3,LOW);      // 4.55 uS
  PORTB |= 1<<4; // Set bit 4 high
    // read PORTD and PORTB
  count[2] = (PORTD & B11111100) | ( PORTB & B00000011);
  //digitalWrite(oe3,HIGH);     // 3.95 uS
  PORTB &= ~(1<<4); // Set bit 4 low
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
   
    if ( delta > 127 ){  count[i] = count[i] - 255; } // counter overflow assume one cycle
   
    if ( delta < -127){  count[i] = count[i] + 255;  } // counter overflow assume one cycle
   
    position[i] = position[i] + delta; 
    
  }
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

void loop() {

    readCounters();
    updatePosition();
        
    Serial.print(F("X"));
    Serial.print((long)position[0]);
    Serial.print(F(";"));

    Serial.print(F("Z"));
    Serial.print((long)position[1]);
    Serial.print(F(";"));

    Serial.print(F("T"));
    Serial.print((unsigned long)position[2]);//tachReadoutMicrosec
    Serial.print(F("/"));
    Serial.print((unsigned long)position[2]); // tach position

  
}
