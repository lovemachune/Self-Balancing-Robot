#include <BalanbotMotor.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Kalman.h>

#define pi 3.1415926

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
double tKP,tKI,tKD;

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
int mode = BalanbotMotor::QUICK_POSITION_CONTROL; //QUICK_POSITION_CONTROL PHI_CONTROL
double reference = 0.002;
double theta_reference = 0; 
double interror = 0;
double olderror = 0;
/*
double kpA = 18, kiA = 107, kdA = 0.58;
double kpB = 18, kiB = 107, kdB = 0.56;
*/
//double kpA = 18, kiA = 200, kdA = 0.3;//PHI_AND_THETA_CONTRO要重調
double kpA = 18, kiA = 180, kdA = 0.2;
double kpB = 18, kiB = 200, kdB = 0.56;
double theta_kpA = -2, theta_kiA = 0, theta_kdA = -0.03;
double theta_kpB = -0.065, theta_kiB = 0, theta_kdB = -0.001;

double peffort,ieffort,deffort,effort;
double wpeffort,wieffort,wdeffort,weffort;
int power;
double phi,wheelAngleB,wheelAngleA;

int time_count,STEP,NEXT_STEP;
const int STEP_PATTERN[] = {1,2,1,2,1,1,4,5,6};
bool ChangeSTEP;
void TimerInterrupt()
{
    sei();
    //讀取的theta和phi角度(非徑度)
    //lastAngleA = currentAngleA;
    //lastAngleB = currentAngleB;
    currentAngleA = motorA.GetWheelAngle();
    currentAngleB = motorB.GetWheelAngle();
    
    //正電壓B輪為正角
    //角度轉徑度
    phi = robotAngle()*pi/180;
    wheelAngleB = currentAngleB*pi/180;
    wheelAngleA = -currentAngleA*pi/180;
    //motorA.GetEffort(peffort,ieffort,deffort,effort,wpeffort,wieffort,wdeffort,weffort,power);
    //myBT.println(" " + String(peffort) + " " + String(ieffort) + " " + String(deffort)+ " " + String(effort)+ " " +String(power));//+ " " + " " + String(wpeffort) + " " + String(wieffort) + " " + String(wdeffort)+ " " + String(weffort) + " " +String(power));
    //motorA.Update(phi,wheelAngleB);
    //motorB.Update(phi,wheelAngleB);
    

    switch(STEP){
      case 0: //take a break
        if(ChangeSTEP){
          ChangeSTEP = false;
          mode = BalanbotMotor::QUICK_POSITION_CONTROL;
          theta_reference = 0;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        time_count=time_count+1;
        if(time_count == 300)        //3s
        {
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
          theta_reference = 6*pi;
          motorA.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA,theta_reference,theta_kpA, theta_kiA, theta_kdA);
        }
        if(wheelAngleA-wheelAngleB<0.01)
          motorA.SetDefaultPWM(10);
        else if(wheelAngleA-wheelAngleB>0.01)
          motorA.SetDefaultPWM(-10);
        else
          motorA.SetDefaultPWM(0);
          
        if( ((wheelAngleB - theta_reference < 0.3*pi)&(wheelAngleB - theta_reference > 0)) || ((wheelAngleB - theta_reference > -0.3*pi)&(wheelAngleB - theta_reference < 0)))
        {
          ChangeSTEP = true;
          STEP = 0;
        }
        break;
      case 2:
        if(ChangeSTEP){
          ChangeSTEP=false;
          mode = BalanbotMotor::PHI_CONTROL;
          motorA.SetControl(mode, reference,kpA,kiA,kdA);
          motorB.SetControl(mode, reference,kpA,kiA,kdA);
          motorA.SetDefaultPWM(-30);
          motorB.SetDefaultPWM(30);
        }
        if(wheelAngleB-wheelAngleA > 0.036)
        {
          ChangeSTEP = true;
          STEP = 0;
        }
        break;
    }
    myBT.println(String(wheelAngleB)+" "+String(NEXT_STEP));
    motorA.Update(phi,wheelAngleB);
    motorB.Update(phi,wheelAngleB);
}

void setup() {
  mpuSetup();
  MsTimer2::set(dT*1000, TimerInterrupt);
  MsTimer2::start();
  setMotor();
  Serial.begin(57600);
  myBT.begin(57600);
  //motorA.Rotate(255);
  //motorB.Rotate(255);
  time_count =0;
  STEP=0;
  NEXT_STEP=0;
  ChangeSTEP=true;
}
int i;
void loop() {
  // put your main code here, to run repeatedly:
  //btData();
}
