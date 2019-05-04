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
float reference = -0.5*pi/180;
double interror = 0 ;
double olderror = 0;
float kp = 30, ki = 0.015, kd = 0.03;

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
    Serial.println(phi);
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
  if(myBT.available())
  {
    key = myBT.read();
    Serial.println(key);
    move(key);
  }
}
