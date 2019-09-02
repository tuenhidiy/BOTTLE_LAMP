//************************************************************************************************************//
#include <SPI.h>

#define blank_pin   3
#define latch_pin   2
#define clock_pin   13
#define data_pin    11

#define ROW0        4
#define ROW1        5
#define BUTTON      12
#define POTPIN      7

//************************************************************************************************************//

byte red[4][2];
byte green[4][2];
byte blue[4][2];

int row=0;
int BAM_Bit, BAM_Counter=0; 

//************************************************************************************************************//
#define BAM_RESOLUTION 4  
#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;

#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473

int buttonPushCounter = 0;   
int buttonState = 0;         
int lastButtonState = 0; 
int POT;
int OLD_POT;

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 8;

pinMode(latch_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(ROW0, OUTPUT);
pinMode(ROW1, OUTPUT);

pinMode(BUTTON, INPUT_PULLUP);

SPI.begin();
interrupts();
fill_colour_wheel();
clearfast();
}

void loop()
{      
buttonState = digitalRead(BUTTON);
 
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
    }
    else {

    }
  }
  lastButtonState = buttonState;

  switch (buttonPushCounter % 2) 
  {
    case 0:

      POT = map(analogRead(POTPIN), 0, 1023, 0, 255);
      if (POT < (OLD_POT * 0.98) || POT > (OLD_POT * 1.02))
        {
          OLD_POT = POT;    
          fillTable_colorwheelRGB(POT, R, G, B);    
        }

        break;
    case 1:

      POT = map(analogRead(POTPIN), 0, 1023, 0, 255);
      if (POT < (OLD_POT * 0.98) || POT > (OLD_POT * 1.02))
        {
          OLD_POT = POT;   
          get_colour(POT, &R, &G, &B); 
          fillTable(R, G, B);   
        }

        break;       
  }
}

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 1);
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));
    
    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  } 
}

void rowScan(byte row)
{  
  if (row == 0) PORTD |= _BV(4);
    else PORTD &= ~_BV(4);
  
  if (row == 1) PORTD |= _BV(5);
    else PORTD &= ~_BV(5);
}


ISR(TIMER1_COMPA_vect){
  
PORTD |= ((1<<blank_pin));

if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      
      SPI.transfer(red[0][row]);
      SPI.transfer(green[0][row]);   
      SPI.transfer(blue[0][row]); 
      break;
    case 1:
      SPI.transfer(red[1][row]);
      SPI.transfer(green[1][row]);   
      SPI.transfer(blue[1][row]);         
      break;
    case 2:     
      SPI.transfer(red[2][row]);
      SPI.transfer(green[2][row]);   
      SPI.transfer(blue[2][row]); 
      break;
    case 3:
      SPI.transfer(red[3][row]);
      SPI.transfer(green[3][row]);   
      SPI.transfer(blue[3][row]); 
        
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

PORTD |= 1<<latch_pin;
PORTD &= ~(1<<latch_pin);
PORTD &= ~(1<<blank_pin);
row++;
if(row==2)
row=0;
pinMode(blank_pin, OUTPUT);
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 2);
    memset(green, 0, sizeof(green[0][0]) * 4 * 2);
    memset(blue, 0, sizeof(blue[0][0]) * 4 * 2);
        
}

void fillTable(byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {
      for (byte y=0; y<8; y++)
      {
        LED(x, y, R, G, B);
      }
    }
}

void fillTable_colorwheelRGB(int potentio, byte R, byte G, byte B)
{
  for (byte x=0; x<2; x++)
    {      
      for (byte y=0; y<8; y++)
      {
        get_colour(potentio + 36*(y+2*x), &R, &G, &B);
        LED(x, y, R, G, B);      
      }
    }
}  

//************************************************************************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}
