#include <BalanbotMotor.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Kalman.h>

#define pi 3.14159

//bluetooth
#define SERIAL_BAUD_RATE  38400
#define BT_RX 12
#define BT_TX 13
#define BT_SPECIFICATION "HC05"
//Bluetooth myBT(BT_RX, BT_TX, BT_SPECIFICATION);
SoftwareSerial myBT(12,13);
char val;
String bluetooth_data = "";
bool inReceive = false,can_set=false;
double tKP,tKI,tKD,temp_R;

//create Motor
BalanbotMotor motorA;
BalanbotMotor motorB;

int LEFT = 1;
int RIGHT = 2;
float maxpowerRemap = 255/100;
float currentAngleA=0, lastAngleA=0, currentAngleB=0, lastAngleB=0;
float dT=0.01;
float speedA, speedB;
char key;

//pid
int mode = BalanbotMotor::PHI_CONTROL; //QUICK_POSITION_CONTROL PHI_CONTROL
double reference =   0.018*0.5;             //0.018*1.5;//0.018*0.6//0.018*-0.3;
double theta_reference = 0; 
double interror = 0;
double olderror = 0;

//double kpA = 22, kiA = 248, kdA = 0.49;
//double kpA = 25, kiA = 228, kdA = 0.52; best
double kpA = 25, kiA = 240, kdA = 0.4;
double kpB = 18, kiB = 200, kdB = 0.56;
double theta_kpA = 0.01, theta_kiA = 0, theta_kdA = 0.001;
//double theta_kpB = 0.05, theta_kiB = 0, theta_kdB = 0.008;

double peffort,ieffort,deffort,effort;
double wpeffort,wieffort,wdeffort,weffort;
int power;
double phi,wheelAngleB,wheelAngleA;

int time_count,REST_TIME;
int STEP,NEXT_STEP;
double angle_temp;
const int STEP_PATTERN[] =  {1,2,1,2,7,3,4,5,6,0,0,0,0,0,0,0,0};
bool ChangeSTEP,BALANCE;
void TimerInterrupt()
{
    sei();
    //讀取的theta和phi角度(非徑度)
    //lastAngleA = currentAngleA;
    //lastAngleB = currentAngleB;    
    
    //正電壓B輪為正角 B右輪 A左輪
    //角度轉徑度
    phi = robotAngle()*pi/180;
    currentAngleA = motorA.GetWheelAngle();
    currentAngleB = motorB.GetWheelAngle();
    wheelAngleB = currentAngleB*pi/180;
    wheelAngleA = -currentAngleA*pi/180;
    /*
    switch(STEP){
      case 0: //take a break
        if(ChangeSTEP){
          ChangeSTEP = false;
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = 0;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        if(wheelAngleA-wheelAngleB < -0.1)
          motorA.SetDefaultPWM(5);
        else if(wheelAngleA-wheelAngleB > 0.1)
          motorA.SetDefaultPWM(-5);
        else
          motorA.SetDefaultPWM(0);
        
        if(wheelAngleB > pi || wheelAngleB < -pi)
          time_count = 0;
        time_count=time_count+1;
        if(time_count == REST_TIME)       //3s
        {
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          time_count=0;
          ChangeSTEP = true;
          motorA.clears();
          motorB.clears();
          STEP = STEP_PATTERN[NEXT_STEP];
          NEXT_STEP = NEXT_STEP + 1;
        }
        break;
      case 1: //straight line
        if(ChangeSTEP){
          ChangeSTEP=false;
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = 10*pi;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        if(wheelAngleA-wheelAngleB < -0.01)
          motorA.SetDefaultPWM(5);
        else if(wheelAngleA-wheelAngleB > 0.01)
          motorA.SetDefaultPWM(-5);
        else
          motorA.SetDefaultPWM(0);
          
        if( ((wheelAngleB - theta_reference < 0.1*pi)&(wheelAngleB - theta_reference > 0)) || ((wheelAngleB - theta_reference > -0.1*pi)&(wheelAngleB - theta_reference < 0)))
        {
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 300;
        }
        break;
      case 2: //左旋90
        if(ChangeSTEP && BALANCE){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorA.SetDefaultPWM(-20);
          motorB.SetDefaultPWM(20);
        }
        if( BALANCE && ((wheelAngleB+wheelAngleA)/2 > 2.5 || (wheelAngleB+wheelAngleA)/2 < -2.5))
        {
          BALANCE = false;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = (wheelAngleB-wheelAngleA)/2;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        else if(!((wheelAngleB+wheelAngleA)/2 > 2.5 || (wheelAngleB+wheelAngleA)/2 < -2.5))
          BALANCE = true;
          
        if(wheelAngleB-wheelAngleA > 7.5)  
        {
          BALANCE = true;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 300;
        }
        break;
      case 3: //右旋180
        if(ChangeSTEP && BALANCE){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorA.SetDefaultPWM(30);
          motorB.SetDefaultPWM(-30);
        }
        if( BALANCE && ((wheelAngleB+wheelAngleA)/2 > 1.6 || (wheelAngleB+wheelAngleA)/2 < -1.6))
        {
          BALANCE = false;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = (wheelAngleB-wheelAngleA)/2;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        else if(!((wheelAngleB+wheelAngleA)/2 > 1.6 || (wheelAngleB+wheelAngleA)/2 < -1.6))
          BALANCE = true;
          
        if(wheelAngleB-wheelAngleA < -15.5)  
        {
          BALANCE = true;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 300;
        }
        break;
      case 4: //順時大圓
        if(ChangeSTEP){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          reference = 0.025;
          theta_reference = 15*pi;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        if(wheelAngleB-wheelAngleA < -7.5)  
        {
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
        }
        else if( (wheelAngleA-wheelAngleB) < 0.31*wheelAngleB)
          motorA.SetDefaultPWM(7);
        else
          motorA.SetDefaultPWM(-10);
          
        
        if(((wheelAngleB - theta_reference < 0.1*pi)&(wheelAngleB - theta_reference > 0)) || ((wheelAngleB - theta_reference > -0.1*pi)&(wheelAngleB - theta_reference < 0)))
        {
          ChangeSTEP = true;
          reference = 0.018*0.6;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 200;
        }
        break;
      case 5: //右旋90
        if(ChangeSTEP && BALANCE){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorA.SetDefaultPWM(20);
          motorB.SetDefaultPWM(-20);
        }
        if( BALANCE && ((wheelAngleB+wheelAngleA)/2 > 1.6 || (wheelAngleB+wheelAngleA)/2 < -1.6))
        {
          BALANCE = false;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = (wheelAngleB-wheelAngleA)/2;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        else if(!((wheelAngleB+wheelAngleA)/2 > 1.6 || (wheelAngleB+wheelAngleA)/2 < -1.6))
          BALANCE = true;
          
        if(wheelAngleB-wheelAngleA < -7.5)  
        {
          BALANCE = true;
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 300;
        }
        break;
      case 6: //順時小圓
        if(ChangeSTEP){
          ChangeSTEP=false;
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = 6*pi;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        if( (wheelAngleA-wheelAngleB) < 0.725*wheelAngleB)
          motorA.SetDefaultPWM(10);
        else
          motorA.SetDefaultPWM(-12);
        if(wheelAngleB-wheelAngleA < -7.5)  
        {
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorB.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          STEP = 0;
          REST_TIME = 500;
        }
        break;
       case 7: //straight line
        if(ChangeSTEP){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          reference = 0.025;
          theta_reference = 10*pi;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,0,0,0,0);
        }
        if(wheelAngleA-wheelAngleB < -0.1)
          motorA.SetDefaultPWM(5);
        else if(wheelAngleA-wheelAngleB > 0.1)
          motorA.SetDefaultPWM(-7);
        else
          motorA.SetDefaultPWM(0);
          
        if( ((wheelAngleB - theta_reference < 0.1*pi)&(wheelAngleB - theta_reference > 0)) || ((wheelAngleB - theta_reference > -0.1*pi)&(wheelAngleB - theta_reference < 0)))
        {
          ChangeSTEP = true;
          motorA.SetDefaultPWM(0);
          motorA.clears();
          motorB.clears();
          reference = 0.018*0.6;
          STEP = 0;
          REST_TIME = 10;
        }
        break;
    }
    */
    motorA.Update(phi,wheelAngleB);
    motorB.Update(phi,wheelAngleB);
    //Serial.println(String(phi));
    //Serial.println(String(wheelAngleA)+" "+String(wheelAngleB));
} 

void setup() {
  mpuSetup();
  MsTimer2::set(dT*500, TimerInterrupt);
  MsTimer2::start();
  setMotor();
  Serial.begin(57600);
  myBT.begin(57600);
  time_count =0;
  REST_TIME = 300;
  STEP=0;
  NEXT_STEP=0;
  ChangeSTEP=true;
  BALANCE = true;
}
int i;
void loop() {
  // put your main code here, to run repeatedly:
  //btData();
  while (myBT.available()) {
  inReceive = true;
  val = BT2.read();
  bluetooth_data += val;
  }
  if(inReceive)
  {
    inReceive = false;
    Serial.println(bluetooth_data);
    bluetooth_data = "";
  }
}
