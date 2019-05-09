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
bool inReceive = false;

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
int mode = 0;
double reference = 0.1*pi/180;
double interror = 0 ;
double olderror = 0;
/*double kpA = 665.4, kiA = 2460.1, kdA = 32.9;
double kpB = 660.4, kiB = 2460.1, kdB = 32.8;*/
double kpA = 18, kiA = 107, kdA = 0.58;
double kpB = 18, kiB = 107, kdB = 0.56;

void TimerInterrupt()
{
    sei();
    lastAngleA = currentAngleA;
    lastAngleB = currentAngleB;
    currentAngleA = motorA.GetWheelAngle();
    currentAngleB = motorB.GetWheelAngle();
    speedA = (currentAngleA - lastAngleA)/dT/360; 
    speedB = (currentAngleB - lastAngleB)/dT/360; 
    double phi = robotAngle()*pi/180;
    //Serial.println(phi);
    motorA.Update(phi);
    motorB.Update(phi);
}

void setup() {
  mpuSetup();
  MsTimer2::set(dT*1000, TimerInterrupt);
  MsTimer2::start();
  setMotor();
  Serial.begin(57600);
  myBT.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  btData();
}
