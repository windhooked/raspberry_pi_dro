
//                      Access bits like variables:
struct bits {
  uint8_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
} __attribute__((__packed__));
#define SBIT_(port,pin) ((*(volatile struct bits*)&port).b##pin)
#define SBIT(x,y)       SBIT_(x,y)

#define SCL A4
#define SDA A5


#define OE1 SBIT( PORTB, 2 )
#define OE2 SBIT( PORTB, 2 )
#define OE3 SBIT( PORTB, 2 )

#define RST1 SBIT( PORTB, 2 )
#define RST2 SBIT( PORTB, 2 )
#define RST3 SBIT( PORTB, 2 )

#define SEL SBIT( PORTB, 2 )


void setup() 
{ 
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
} 
 
// Interrupt is called once a millisecond 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  
   OE1 = 0;  
   counters[0] = read();
   OE1 = 1;  
   RST1 = 0; RST1 = 1;
   
   OE2 = 0;  
   counters[1] = read();
   OE2 = 1;  
   RST2 = 0; RST2 = 1;
   
   OE3 = 0;  
   counters[2] = read();
   OE3 = 0;  
   RST2 = 0; RST2 = 1;
     
} 

int read() {

  int _data = 0;
  SEL = 0;
  Wire.requestFrom(0x00, 2);
  if(Wire.available()) {
    _data = Wire.receive();
  }
  SEL = 1;
  if(Wire.available()) {
    _data |= (Wire.receive() << 8);
  }
  return _data;
}
 
void loop()
{
}
