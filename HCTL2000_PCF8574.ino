
//                      Access bits like variables:
struct bits {
  uint8_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
} __attribute__((__packed__));
#define SBIT_(port,pin) ((*(volatile struct bits*)&port).b##pin)
#define SBIT(x,y)       SBIT_(x,y)

#define OE1 SBIT( PORTB, 2 )
#define OE2 SBIT( PORTB, 2 )
#define OE3 SBIT( PORTB, 2 )

#define RST1 SBIT( PORTB, 2 )
#define RST2 SBIT( PORTB, 2 )
#define RST3 SBIT( PORTB, 2 )

#define SEL SBIT( PORTB, 2 )

double counters[3];

void setup() 
{ 
  Serial.begin(115200);
  
  // CLK signal for HCTL2000 
  // set up 8 MHz timer on CLOCKOUT (OC1A)
  pinMode (clk, OUTPUT); 
  // set up Timer 1
  TCCR1A = bit (COM1A0);  // toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);   // CTC, no prescaling
  OCR1A = 0; // output every cycle
  
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
} 
 
// Interrupt is called once a millisecond 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long t1 = millis();
  
   OE1 = 0;  
   counters[0] += readHCTL();
   OE1 = 1;  
   RST1 = 0; RST1 = 1;
   
   OE2 = 0;  
   counters[1] += readHCTL();
   OE2 = 1;  
   RST2 = 0; RST2 = 1;
   
   OE3 = 0;  
   counters[2] += readHCTL();
   OE3 = 0;  
   RST2 = 0; RST2 = 1;
   
  unsigned long t2 = millis();
     

int readHCTL() {

  unsigned char _l = 0;
  unsigned char _h = 0;
  int _data = 0;
  
  SEL = 1;
  
  Wire.requestFr(0x00, 2);
  if(Wire.available()) {
    _l = Wire.receive();
    _data = _l;
  }
  SEL = 0;
  if(Wire.available()) {
    _h = Wire.receive();
    _data |= ( _h << 8 )
  }
   // https://www.avrfreaks.net/forum/any-neat-sign-extension-tricks-10-bit-16-bit
   //struct {signed int _data:12;} x;
   //_data = x._data = _data;
   _data = ( _data ^ 0x800 ) - 0x800;
   
  //     _data = ( x>> 11) == 0 ? _data : -1 ^ 0xFFF | _data;
   // _data = ( _data & 0x800 ? _data | 0xf000 : _data );
    
    //if  ( _h & (1<<4) ) { // test for sign, negative?
    // _data = 0xf000; // add sign to 16 bit int
    // clearbit(_h,4);  // clear sign on 12 bit sample
    // _data | = _h<<8; // load high byte to 16bit int
    //} 
   
  }
  return _data;
}
 
void loop()
{
Serial.print("X: ");
Serial.println(counters[0]);
Serial.print("t: ");
Serial.println(t2-t1);
 
}
