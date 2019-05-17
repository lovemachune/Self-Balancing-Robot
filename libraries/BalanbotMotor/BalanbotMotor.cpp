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

void BalanbotMotor::SetControl(int mode, double reference, double kp, double ki, double kd,double theta_reference=0,double theta_kp=0,double theta_ki=0,double theta_kd=0)
{
  //TODO
  mControlMode = mode;
  if(mControlMode == PHI_CONTROL)
  {
    angleController.SetPID(kp,ki,kd);
    angleController.SetReference(reference);
  }
  else if(mControlMode == PHI_AND_THETA_CONTROL)
  {
    angleController.SetPID(kp,ki,kd);
    angleController.SetReference(reference);
    speedController.SetPID(theta_kp,theta_ki,theta_kd);
    speedController.SetReference(theta_reference);
  }
}

void BalanbotMotor::InverseRotationDirectionDefinition(const bool ifInverse){
  if( ifInverse )
    mDirectionCoefficient = -1.0;
  else
    mDirectionCoefficient = 1.0;
}


void BalanbotMotor::Rotate(int voltage){
  //TODO
  bool inPin1 = HIGH, inPin2 = LOW;
  digitalWrite(mStandbyPin, HIGH);    
  if(voltage > 255)
  {
    voltage = 255;
  }
  else if(mDirectionCoefficient*voltage < 0)
  {
    inPin1 = LOW;
    inPin2 = HIGH;
    if(voltage < -255)
      voltage = 255;
    else
      voltage = -voltage;
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
            * ( static_cast<double>(encoderPosition) 
              / static_cast<double>(mEncoder.GetPPR()) );
}

void BalanbotMotor::UpdateSpeed(){
  mSpeed = mDifferentiator.differential(mAngle);
}

void BalanbotMotor::UpdateEncoder(){
  mEncoder.Update();
}

void BalanbotMotor::UpdateControl(double phi,double theta)
{
  //TODO
  int power;
  if(mControlMode == PHI_CONTROL)
    power = (int)angleController.Update(phi);
  else if(mControlMode == PHI_AND_THETA_CONTROL)
    power = (int)(0.7*angleController.Update(phi))+(int)(0.3*speedController.Update(theta));
  Rotate(-power);
}

void BalanbotMotor::Update(double phi,double theta=0){
  UpdateAngle();
  UpdateSpeed();
  UpdateControl(phi,theta);
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

int BalanbotMotor::GetEncoderInterruptPin() 
{ 
  return mEncoder.GetInterruptPin();
}

double BalanbotMotor::GetSpeed() 
{
  return mSpeed;
} 

double BalanbotMotor::GetAngle() 
{
  return mAngle;
}
void BalanbotMotor::GetEffort(double &pEffort ,double &iEffort ,double &dEffort ,double &effort)
{
  angleController.GetEffort(pEffort ,iEffort ,dEffort,effort);
}
int BalanbotMotor::GetWheelAngle()
{
  double count = mEncoder.GetCount();
  return 2*PI*(count/SECTION_NUMBER)*RAD_TO_DEG;
}