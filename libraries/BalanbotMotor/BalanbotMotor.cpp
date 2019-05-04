#include "BalanbotMotor.h"


BalanbotMotor::BalanbotMotor() :
  mDirectionCoefficient (1.0),
  mSpeed (0.0),
  mAngle (0.0),
  mControlMode (0)
{

}

inline void BalanbotMotor::SetPWMPin(const int pin)
{
  mPwmPin = pin;
  //TODO
  pinMode(mPwmPin, OUTPUT);
}

inline void BalanbotMotor::SetDirectionPins( const int pinA, 
                                             const int pinB )
{
  mDirectionPinA = pinA;
  mDirectionPinB = pinB; 
  //TODO
  pinMode(mDirectionPinA, OUTPUT);
  pinMode(mDirectionPinB, OUTPUT);
}

inline void BalanbotMotor::SetStandbyPin(const int pin)
{
  mStandbyPin = pin;
  //TODO
  pinMode(mStandbyPin, HIGH);
}

void BalanbotMotor::SetMotorPins( const int pwmPin, 
                                  const int directionPinA, 
                                  const int directionPinB, 
                                  const int standbyPin )
{
  SetPWMPin(pwmPin);
  SetDirectionPins(directionPinA, directionPinB);
  SetStandbyPin(standbyPin);
}

void BalanbotMotor::SetEncoderPins( const int interruptPin, 
                                    const int directionPin )
{
  mEncoder.SetInterruptPin(interruptPin);
  mEncoder.SetDirectionPin(directionPin); 
}

void BalanbotMotor::SetControl(int mode, float reference, float kp, float ki, float kd)
{
  //TODO
  angleController.SetPID(kp,ki,kd);
  angleController.SetReference(reference);
}

void BalanbotMotor::InverseRotationDirectionDefinition(const bool ifInverse){
  if( ifInverse )
    mDirectionCoefficient = -1.0;
  else
    mDirectionCoefficient = 1.0;
}

int BalanbotMotor::GetEncoderInterruptPin() 
{ 
  return mEncoder.GetInterruptPin();
}

float BalanbotMotor::GetSpeed() 
{
  return mSpeed;
} 

float BalanbotMotor::GetAngle() 
{
  return mAngle;
}

void BalanbotMotor::Rotate(const int voltage){
  //TODO
  bool inPin1 = HIGH, inPin2 = LOW;
  digitalWrite(mStandbyPin, HIGH);    
  if(mDirectionCoefficient*voltage < 0)
  {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  digitalWrite(mDirectionPinA, inPin1);
  digitalWrite(mDirectionPinB, inPin2);
  analogWrite(mPwmPin, voltage);
}

void BalanbotMotor::Brake(){
  //TODO
  digitalWrite(mStandbyPin, LOW);
}

void BalanbotMotor::UpdateAngle(){
  int encoderPosition = mEncoder.GetPosition();
  mAngle =  mDirectionCoefficient 
            * (2*PI) 
            * ( static_cast<float>(encoderPosition) 
              / static_cast<float>(mEncoder.GetPPR()) );
}

void BalanbotMotor::UpdateSpeed(){
  mSpeed = mDifferentiator.differential(mAngle);
}

void BalanbotMotor::UpdateEncoder(){
  mEncoder.Update();
}

void BalanbotMotor::UpdateControl(float phi)
{
  //TODO
  int power = (int)angleController.Update(phi);
  Rotate(-power);
}

void BalanbotMotor::Update(float phi){
  UpdateAngle();
  UpdateSpeed();
  UpdateControl(phi);
}
int BalanbotMotor::GetWheelAngle()
{
  float count = mEncoder.GetCount();
  return 2*PI*(count/SECTION_NUMBER)*RAD_TO_DEG;
}

void BalanbotMotor::move(int speed, int direction)
{
  bool inPin1 = HIGH, inPin2 = LOW;
  digitalWrite(mStandbyPin, HIGH);    
  if(direction==1)
  {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  digitalWrite(mDirectionPinA, inPin1);
  digitalWrite(mDirectionPinB, inPin2);
  analogWrite(mPwmPin, speed);
}
void BalanbotMotor::clears()
{
  mEncoder.clear();
}