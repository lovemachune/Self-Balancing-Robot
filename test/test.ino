#include <BalanbotMotor.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h> 
#include <Wire.h>

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
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;


int LEFT = 1;
int RIGHT = 2;
float maxPowerRemap = 255/100;
float currentAngleA=0, lastAngleA=0, currentAngleB=0, lastAngleB=0;
float dT=0.01;
float speedA, speedB;
char key;

void TimerInterrupt()
{
    sei();
    lastAngleA = currentAngleA;
    currentAngleA = motorA.GetWheelAngle();
    lastAngleB = currentAngleB;
    currentAngleB = motorB.GetWheelAngle();
    speedA = (currentAngleA - lastAngleA)/dT/360; 
    speedB = (currentAngleB - lastAngleB)/dT/360; 
}

void move(char key)
{
  switch(key)
  {
    case 'w':
      motorA.move(50,1);
      motorB.move(50,1);
      break;
    case 'a':
      motorA.move(85,1);
      motorB.move(50,1);
      break;
    case 's':
      motorA.move(50,0);
      motorB.move(50,0);
      break;
    case 'd':
      motorA.move(50,1);
      motorB.move(85,1);
      break;
    case 'x':
      motorA.stop();
      motorB.stop();
      break; 
  }
}

void updateA()
{
  motorA.Update();
}
void updateB()
{
  motorB.Update();
}

void setup() {
  // put your setup code here, to run once:
  //initMotor();  
  //myBT.Begin();
  mpuSetup();
  MsTimer2::set(dT*1000, TimerInterrupt);
  MsTimer2::start();
  motorA.SetMotorPins(PWMA, AIN1, AIN2, STBY);
  motorA.SetEncoderPins(A_Interrupt, A_Direction);
  motorB.SetMotorPins(PWMB, BIN1, BIN2, STBY);
  motorB.SetEncoderPins(B_Interrupt, B_Direction);
  attachInterrupt(digitalPinToInterrupt(motorA.GetEncoderInterruptPin()), updateA, RISING);
  attachInterrupt(digitalPinToInterrupt(motorB.GetEncoderInterruptPin()), updateB, RISING);
  Serial.begin(9600);
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
  
  data_sent();
  delay(1000);  
}
