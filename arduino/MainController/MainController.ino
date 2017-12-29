#include "MsTimer2.h"
#include "MatrixMath.h"
#include "Acrobot.h"
#include <SPI.h>

// Slave 핀맵(ServoController1, 2), SPI 통신
int iSS_Servo1 = 10;
int iSS_Servo2 = 9;

// SPI 통신 Data
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

// SamplingTime(단위:ms)
#define SamplingTime_Main 5

// 단위 변환
#define msTOs 0.001
#define NmTomA 1

// 최대 모터 갯수
#define MaxServoNum 2

// 최대 모터 속도(deg/s)
#define MaxServoSpeed 180

int i = 0;

// 제어 모드 변수
typedef struct{
  int iServoID;
}MODE;

MODE g_SwingUpMode;
MODE g_PositionMode;

int g_iInit = 0;

// 제어 완료 상태 Flag
int g_iPositionControllerFlag = 0;
int g_iSwingUpControllerFlag = 0;

// 그리퍼 상태 Flag
int g_iGrab1Flag = 0;
int g_iGrab2Flag = 0;

// 버튼 상태 Flag
int g_iButtonStart = 0;

// 제어 관련 변수
float g_fTime_s = 0.0;
float g_fTime_ms = 0.0;
float g_fActualPos[MaxServoNum] = {0.0, 0.0};
float g_fActualPos_Old[MaxServoNum] = {0.0, 0.0};
float g_fActualVel[MaxServoNum] = {0.0, 0.0};
float g_fActualVel_Old[MaxServoNum] = {0.0, 0.0};
float g_fActualAcc[MaxServoNum] = {0.0, 0.0};
float g_fReferencePos[MaxServoNum] = {0.0, 0.0};
float g_fReferenceVel[MaxServoNum] = {0.0, 0.0};  
float g_fInputCurrentCmd[MaxServoNum] = {0.0, 0.0};
float g_fInputCurrentCmd_Old[MaxServoNum] = {0.0, 0.0};

// Trajectory 생성 관련 변수
float g_fA0 = 0.0;
float g_fA1 = 0.0;
float g_fA2 = 0.0;
float g_fA3 = 0.0;
float g_fEndTime = 0.0;
float g_fStartPos = 0.0;
float g_fGoalPos = 0.0;

// SwingUp 제어 관련 변수
float g_VecQ_Servo1[2] = {0, 0};
float g_VecQdot_Servo1[2] = {0, 0};
float g_VecQ_Servo2[2] = {0, 0};
float g_VecQdot_Servo2[2] = {0, 0};

float g_vecDesQ_Servo1[2] = {0, 0};
float g_vecDesQdot_Servo1[2] = {0, 0};
float g_vecDesQddot_Servo1[2] = {0, 0};
float g_vecDesQ_Servo2[2] = {0, 0};
float g_vecDesQdot_Servo2[2] = {0, 0};
float g_vecDesQddot_Servo2[2] = {0, 0};

Link link1(0.1710, 0.377790, 0.27948, 0.0027273, 0.01);
Link link2(0.289, 0.388200, 0.32843, 0.003348, 0.01);

Acrobot acrobot[2];

// Time Delay Control Gain tunning 
float g_fMbar[MaxServoNum] = {0.0, 0.0};

void setup() 
{ 
  // Master 설정(MainController), SPI 통신
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(iSS_Servo1, OUTPUT);
  pinMode(iSS_Servo2, OUTPUT);
  digitalWrite(iSS_Servo1, HIGH);
  digitalWrite(iSS_Servo2, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // 시리얼 통신 초기화
  Serial.begin (9600);

  // SmaplingTime_Main(단위:ms) 마다 제어
  MsTimer2::set(SamplingTime_Main,MainController);
  MsTimer2::start();

  // SwingUp 관련 초기화
  acrobot[0].setLinkInform(link1, link2);
  acrobot[1].setLinkInform(link1, link2);

  acrobot[0].setTargetValue(g_vecDesQ_Servo1, g_vecDesQdot_Servo1, g_vecDesQddot_Servo1);
  acrobot[1].setTargetValue(g_vecDesQ_Servo2, g_vecDesQdot_Servo2, g_vecDesQddot_Servo2);

  acrobot[0].setParameter4SwingUp(2, 100, 50);
  acrobot[1].setParameter4SwingUp(2, 100, 50);

  // 초기 제어 모드 - None Controller
  g_PositionMode.iServoID = 1;
  g_SwingUpMode.iServoID = 0;
  g_iInit = 1;
}

void loop() 
{ 
  for(i=0; i<MaxServoNum; i++)
  {
    GetActualPosition(i+1);
  }
  
  //Serial.println(g_fActualPos[0]);
  Serial.print("g_fReferencePos[0] : ");Serial.println(g_fReferencePos[0]);
}

byte TransferData(const byte data)
{
  byte temp = SPI.transfer(data);
  delayMicroseconds(30);
  return temp;
}

void GetActualPosition(int ServoNum)
{
  if(ServoNum == 1)
  {
    //g_fActualPos[0] = 360;
    digitalWrite(iSS_Servo1, LOW);
    TransferData('a');
    encoder.b[0] = TransferData('b');
    encoder.b[1] = TransferData('c');
    encoder.b[2] = TransferData('d');
    encoder.b[3] = TransferData('e');
    g_fActualPos[0] = encoder.f;
    digitalWrite(iSS_Servo1, HIGH);
  }
  else if(ServoNum == 2)
  {
    digitalWrite(iSS_Servo2, LOW);
    TransferData('a');
    encoder.b[0] = TransferData('b');
    encoder.b[1] = TransferData('c');
    encoder.b[2] = TransferData('d');
    encoder.b[3] = TransferData('e');
    g_fActualPos[1] = encoder.f;
    digitalWrite(iSS_Servo2, HIGH);
  } 
}

void SetInputCommand(int ServoNum, float InputCurrentCommand)
{
  //InputCurrentCommand = 5.6;
  current.f = InputCurrentCommand;
  
  if(ServoNum == 1)
  {
    digitalWrite(iSS_Servo1, LOW);
    SPI.transfer(current.b[0]);
    delayMicroseconds(30);
    SPI.transfer(current.b[1]);
    delayMicroseconds(30);
    SPI.transfer(current.b[2]);
    delayMicroseconds(30);
    SPI.transfer(current.b[3]);
    delayMicroseconds(30);
    digitalWrite(iSS_Servo1, HIGH);
  }
  else if(ServoNum == 2)
  {
    digitalWrite(iSS_Servo2, LOW);
    SPI.transfer(current.b[0]);
    delayMicroseconds(30);
    SPI.transfer(current.b[1]);
    delayMicroseconds(30);
    SPI.transfer(current.b[2]);
    delayMicroseconds(30);
    SPI.transfer(current.b[3]);
    delayMicroseconds(30);
    digitalWrite(iSS_Servo2, HIGH);
  }
}

void MainController()
{                     
  UpdateControlParameter();

  if(g_iInit == 1 && g_PositionMode.iServoID > 0) // 초기화 자세 셋팅
  { 
    // 원하는 목표점으로 제어
    switch(g_iPositionControllerFlag)
    {
      case 0:
        CalculatePosition(g_PositionMode.iServoID, 1, 90);
        g_iPositionControllerFlag = 1;
        break;
      case 1:
        PositionController(g_PositionMode.iServoID, 1);
        break;
      case 2:
        g_iInit = 0;
        g_PositionMode.iServoID = 0;
        g_SwingUpMode.iServoID = 0;
        g_iPositionControllerFlag = 0;
        break;  
    }    
  }
  else if(g_SwingUpMode.iServoID > 0 && g_PositionMode.iServoID > 0) // SwingUp && 현재 자세 유지
  {
    // Swing Up
    SwingUpController(g_SwingUpMode.iServoID);
    
    // 현재 자세 유지
    switch(g_iPositionControllerFlag)
    {
      case 0:
        g_fReferencePos[g_PositionMode.iServoID-1] = g_fActualPos[g_PositionMode.iServoID-1];
        g_iPositionControllerFlag = 1;
        break;
      case 1:
        PositionController(g_PositionMode.iServoID, 0);
        break;
      case 2:
        if(g_iGrab1Flag == 1)
        {
          g_PositionMode.iServoID = 2;
          g_SwingUpMode.iServoID = 0;
          g_iPositionControllerFlag = 0;
        }
        else if(g_iGrab2Flag == 1)
        {
          g_PositionMode.iServoID = 1;
          g_SwingUpMode.iServoID = 0;
          g_iPositionControllerFlag = 0;          
        }
        break;  
    }
  }
  else if(g_SwingUpMode.iServoID == 0 && g_PositionMode.iServoID > 0) // 원점으로 자세 제어
  {     
    // 원점으로 자세 제어
    switch(g_iPositionControllerFlag)
    {
      case 0:
        CalculatePosition(g_PositionMode.iServoID, 0, 0);
        g_iPositionControllerFlag = 1;
        break;
      case 1:
        PositionController(g_PositionMode.iServoID, 1);
        break;
      case 2:
        if(g_iGrab1Flag == 0)
        {
          g_PositionMode.iServoID = 2;
          g_SwingUpMode.iServoID = 1;
          g_iPositionControllerFlag = 0;
        }
        else if(g_iGrab2Flag == 0)
        {
          g_PositionMode.iServoID = 1;
          g_SwingUpMode.iServoID = 2;
          g_iPositionControllerFlag = 0;          
        }
        break; 
    }
  }
  else if(g_SwingUpMode.iServoID == 0 && g_PositionMode.iServoID == 0) // No Control MODE
  {
    NoneController();
  }
}

void UpdateControlParameter()
{
  int i = 0;

  for(i=0; i<MaxServoNum; i++)
  {
    g_fActualVel[i] = (g_fActualPos[i] - g_fActualPos_Old[i]) / SamplingTime_Main;
    g_fActualAcc[i] = (g_fActualVel[i] - g_fActualVel_Old[i]) / SamplingTime_Main;
    g_fActualPos_Old[i] = g_fActualPos[i];
    g_fActualVel_Old[i] = g_fActualVel[i];
    g_fInputCurrentCmd_Old[i] = g_fInputCurrentCmd[i];
  }

  // Swing Up 관련 파라미터 업데이트
  g_VecQ_Servo1[0]; // IMU 센서값 받아오기
  g_VecQdot_Servo1[0];
  g_VecQ_Servo1[1] = g_fActualPos[0];
  g_VecQdot_Servo1[1] = g_fActualVel[0];

  g_VecQ_Servo1[0];  // IMU 센서값 받아오기
  g_VecQdot_Servo1[0];
  g_VecQ_Servo1[1] = g_fActualPos[1];
  g_VecQdot_Servo1[1] = g_fActualVel[1];  
}

void NoneController()
{
  if(g_iButtonStart == 1)
  {
    g_SwingUpMode.iServoID = 1;
    g_PositionMode.iServoID = 2;
    g_iButtonStart = 0;      
  }
}

void SwingUpController(int ServoNum)
{
  int Select_Servo = ServoNum-1;

  if(ServoNum == 1)
  {
    g_fInputCurrentCmd[Select_Servo] = NmTomA * acrobot[Select_Servo].calcControlInput(g_VecQ_Servo1, g_VecQdot_Servo1);
    SetInputCommand(ServoNum, g_fInputCurrentCmd[Select_Servo]);
  }
  else if(ServoNum == 2)
  { 
    g_fInputCurrentCmd[Select_Servo] = NmTomA * acrobot[Select_Servo].calcControlInput(g_VecQ_Servo2, g_VecQ_Servo2);
    SetInputCommand(ServoNum, g_fInputCurrentCmd[Select_Servo]);
  }
}

// Mode = 0 : 현재 자세 유지, Mode = 1 : 원하는 목표점 위치 제어
void PositionController(int ServoNum, int Mode)
{
  int Select_Servo = ServoNum-1;
  float fDED[MaxServoNum];
  float fTDE[MaxServoNum];
  float fKD[MaxServoNum] = {20.0, 20.0};
  float fKP[MaxServoNum] = {100.0, 100.0};
  float fError[MaxServoNum]; 
  float fErrorDot[MaxServoNum];

  if(Mode == 1)
    TrajectoryGenerator(Select_Servo);     
  
  fError[Select_Servo] = g_fReferencePos[Select_Servo] - g_fActualPos[Select_Servo];
  fErrorDot[Select_Servo] = g_fReferenceVel[Select_Servo] - g_fActualVel[Select_Servo];
  
  fDED[Select_Servo] = g_fReferencePos[Select_Servo] + fKD[Select_Servo] * fErrorDot[Select_Servo] + fKP[Select_Servo] * fError[Select_Servo];
  fTDE[Select_Servo] = g_fInputCurrentCmd_Old[Select_Servo] - g_fMbar[Select_Servo] * g_fActualAcc[Select_Servo];

  g_fInputCurrentCmd[Select_Servo] = g_fMbar[Select_Servo] * fDED[Select_Servo] + fTDE[Select_Servo];
    
  if(ServoNum == 1)
  { 
    SetInputCommand(ServoNum, g_fInputCurrentCmd[0]);
  }
  else if(ServoNum == 2)
  {
    SetInputCommand(ServoNum, g_fInputCurrentCmd[1]);
  }
}

// 3차 다항식 Trajectory 현재 위치를 기준으로 g_fGoalPos로 이동
void TrajectoryGenerator(int Select_Servo)
{
  int iScaleFactor = 2;
  
  g_fTime_ms += SamplingTime_Main;
  g_fTime_s = g_fTime_ms * msTOs;
    
  g_fEndTime = iScaleFactor * 2*abs(g_fGoalPos-g_fStartPos)/MaxServoSpeed;

  if(g_fEndTime == 0.0)
  {
    g_fA0 = g_fStartPos;
    g_fA1 = 0.0;
    g_fA2 = 0.0;
    g_fA3 = 0.0;        
  }
  else
  {
    g_fA0 = g_fStartPos;
    g_fA1 = 0.0;
    g_fA2 = 3/(g_fEndTime*g_fEndTime)*(g_fGoalPos-g_fStartPos);
    g_fA3 = -2/(g_fEndTime*g_fEndTime*g_fEndTime)*(g_fGoalPos-g_fStartPos);    
  }
  
  g_fReferencePos[Select_Servo] = g_fA0 + g_fA1*g_fTime_s + g_fA2*g_fTime_s*g_fTime_s + g_fA3*g_fTime_s*g_fTime_s*g_fTime_s;
  g_fReferenceVel[Select_Servo] = g_fA1 + 2*g_fA2*g_fTime_s + 3*g_fA3*g_fTime_s*g_fTime_s;

  if(g_fTime_s > g_fEndTime)
  {
    g_fTime_ms = 0.0;
    g_fTime_s = 0.0;
    g_iPositionControllerFlag = 2;
  }
}

// Mode = 0 원점으로 제어, Mode = 1 임의 목표점 셋팅(PosTemp) 
int CalculatePosition(int ServoNum, int Mode, float PosTemp)
{  
  int Select_Servo = ServoNum-1;
  float fGoalPos;
  float fDirectionPos;
  float fActualPos = g_fActualPos[Select_Servo];
  float fAbsActualPos = abs(fActualPos);

  g_fStartPos = fActualPos;

  if(Mode == 0)
  { 
    if(fActualPos >= 0)
    {
      fDirectionPos = +1.0;
    }
    else
    {
      fDirectionPos = -1.0;
    }
    
    if(fAbsActualPos < 360)
    {
      if(fAbsActualPos >= 180)
      {
        fAbsActualPos = 360 - fAbsActualPos;
        
        if(fDirectionPos > 0)
        {
          g_fGoalPos = fAbsActualPos + g_fStartPos;
        }
        else
        {
          g_fGoalPos = -1 * fAbsActualPos + g_fStartPos;
        }      
      }
      else
      {
        if(fDirectionPos > 0)
        {
          g_fGoalPos = -1 * fAbsActualPos + g_fStartPos;
        }
        else
        {
          g_fGoalPos = fAbsActualPos + g_fStartPos;
        }      
      }
      
      return fGoalPos;  
    }
    else if(fAbsActualPos > 360)
    {
      fAbsActualPos = fAbsActualPos - int(fAbsActualPos/360.0) * 360;
  
      if(fAbsActualPos >= 180)
      {
        fAbsActualPos = 360 - fAbsActualPos;
        
        if(fDirectionPos > 0)
        {
          g_fGoalPos = fAbsActualPos + g_fStartPos;
        }
        else
        {
          g_fGoalPos = -1 * fAbsActualPos + g_fStartPos;
        } 
      }
      else
      {
        if(fDirectionPos > 0)
        {
          g_fGoalPos = -1 * fAbsActualPos + g_fStartPos;
        }
        else
        {
          g_fGoalPos = fAbsActualPos + g_fStartPos;
        }       
      }  
    }
    else
    {
      g_fGoalPos = g_fStartPos;
    } 
  }
  else if(Mode == 1)
  {
    g_fGoalPos = PosTemp;    
  }
}
