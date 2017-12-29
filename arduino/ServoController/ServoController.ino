#include "DualVNH5019MotorShield.h"
#include <SPI.h>

// SamplingTime(단위:ms)
#define SamplingTime_Current 5

// 단위 변환
#define msTOs 0.001
#define pulseToturn 0.0000625 // 1/16000 -> 1바퀴당 16000펄스
#define turnToradian 2*PI
#define radianTodgree 180/(2*PI)

// SPI 통신 관련 변수
int iCount = 0;
byte command = 0;

union Data1
{
  float f;
  byte b[4];
}encoder;

union Data2
{
  float f;
  byte b[4];
}current;

// 모터 관련  
DualVNH5019MotorShield md;

// 엔코더 관련 
#define encoder0PinA  2
#define encoder0PinB  3

volatile unsigned int encoder0Pos = 0.0;
float g_fActualPosition = 0.0;

// 전류 제어 관련
float g_fActCurrent = 0.0;
float g_fRefCurrent = 0.0;
float g_fSum = 0.0;
float g_fError = 0.0;
float g_fInputCommand = 0.0;

float g_fMag = 3.0;
float g_fFreq = 0.5;
float g_fTime_s = 0.0;
float g_fTime_ms = 0.0;

// 전류 Gain(PI제어)
float g_fKp = 0.0;
float g_fKI = 0.0;

void setup() 
{ 
  // Slave 설정(ServoController), SPI 통신
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SCK, INPUT);
  pinMode(SS, INPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);

  // 모터 초기화
  md.init();

  // 엔코더 초기화
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
  
  // 시리얼 통신 초기화
  Serial.begin (9600);
}
 
void loop() 
{
  if(digitalRead(SS) == HIGH)
    command = 'e';
    
  CurrentController();

  //Serial.println(g_fRefCurrent);
}

ISR (SPI_STC_vect)
{
  byte c = SPDR;
        
  if(c != 'e' && c != 'a' && c != 'b' && c != 'c' && c != 'd')
  {
    current.b[iCount] = SPDR;
    iCount++;
    if(iCount > 3)
    {
      g_fRefCurrent = current.f;
      iCount = 0;
    }
  }
  else
  {
    //encoder.f = 15.31;
    g_fActualPosition = encoder0Pos * pulseToturn * turnToradian;
    encoder.f = g_fActualPosition;
    
    command = c; 
  
    switch(command)
    {
      case 'e':
        SPDR = 0;
        break;
      case 'a':
         SPDR = encoder.b[0];
         break;
      case 'b':
        SPDR = encoder.b[1];
        break;
      case 'c':
        SPDR = encoder.b[2];
        break;
      case 'd':
        SPDR = encoder.b[3];
        break;
    }
  }
}

void CurrentController()
{ 
// 전류 튜닝용 --------------------------------------------------------------
/*   
  g_fTime_ms += SamplingTime_Current;
  g_fTime_s = g_fTime_ms * msTOs;
  g_fRefCurrent = g_fMag*sin(2*PI*g_fFreq*g_fTime_s);
*/
// --------------------------------------------------------------------------

  g_fActCurrent = md.getM2CurrentMilliamps();

// 전류 제한(8mA)
  if(abs(g_fActCurrent) > 8)
    g_fInputCommand = 0;
  
  g_fError = g_fRefCurrent - g_fActCurrent;

  g_fSum = g_fSum + g_fError;

  g_fInputCommand = g_fKp * g_fError + g_fKI * g_fSum;
 
  md.setM2Speed(g_fInputCommand);

}

void doEncoderA() {
  
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }  
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}
