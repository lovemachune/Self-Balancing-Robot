#include <BalanbotMotor.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Kalman.h>

#define pi 3.1415926
//motorA define
#define PWMA 5
#define AIN1 6                     //IN1=H, IN2=L -> dir+
#define AIN2 4
#define A_Interrupt 2
#define A_Direction A3

//motorB defiene
#define PWMB 9
#define BIN1 11
#define BIN2 10
#define B_Interrupt 3
#define B_Direction 8

#define STBY 7                    //STBY=H -> enable, STBY=L -> unable

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

//MPU 
#define RESTRICT_PITCH
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t powerRaw, tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


int LEFT = 1;
int RIGHT = 2;
float maxpowerRemap = 255/100;
float currentAngleA=0, lastAngleA=0, currentAngleB=0, lastAngleB=0;
float dT=0.01;
float speedA, speedB;
char key;

//pid
int mode = 0;
float reference = 0;
double interror = 0 ;
double olderror = 0;
float kp = 20, ki = 0.001, kd = 0.1;
double kpS,kiS,kdS;

void TimerInterrupt()
{
    sei();
    lastAngleA = currentAngleA;
    currentAngleA = motorA.GetWheelAngle();
    lastAngleB = currentAngleB;
    currentAngleB = motorB.GetWheelAngle();
    speedA = (currentAngleA - lastAngleA)/dT/360; 
    speedB = (currentAngleB - lastAngleB)/dT/360; 
    robotAngle();
    double phi = kalAngleX;
    Serial.println(phi);
    motorA.Update(phi);
    motorB.Update(phi);
}



/*void pidControl()
{
  double  error = kalAngleX*pi/180;
  interror += error;
  double lasterror = error - olderror;
  olderror = error;
  int power = (error*kp + interror*ki + lasterror*kd);
  if(power>51)
    power = 51;
  if(power<-51)
    power = -51;
  if(power>0)
  {
    motorA.move(power,0);
    motorB.move(power,0);
  }
  else
  {
    motorA.move(-power,1);
    motorB.move(-power,1);
  }
  Serial.print(error);
  Serial.print(" ");
  Serial.println(power);
}*/

void updateA()
{
  motorA.UpdateEncoder();
}
void updateB()
{
  motorB.UpdateEncoder();
}

void setup() {
  mpuSetup();
  MsTimer2::set(dT*1000, TimerInterrupt);
  MsTimer2::start();
  motorA.SetMotorPins(PWMA, AIN1, AIN2, STBY);
  motorA.SetEncoderPins(A_Interrupt, A_Direction);
  motorA.InverseRotationDirectionDefinition(false);
  motorA.SetControl(mode, reference,kp,ki,kd);

  motorB.SetMotorPins(PWMB, BIN1, BIN2, STBY);
  motorB.SetEncoderPins(B_Interrupt, B_Direction);
  motorB.InverseRotationDirectionDefinition(false);
  motorB.SetControl(mode, reference,kp,ki,kd);

  attachInterrupt(digitalPinToInterrupt(motorA.GetEncoderInterruptPin()), updateA, RISING);
  attachInterrupt(digitalPinToInterrupt(motorB.GetEncoderInterruptPin()), updateB, RISING);
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
