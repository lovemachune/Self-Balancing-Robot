#ifndef BALANBOTMOTOR_H
#define BALANBOTMOTOR_H

#include "BalanbotEncoder.h"
#include "BalanbotController.h"
#include <NumericalTool.h>
#include <Arduino.h>

class BalanbotMotor{
  public:
    static const int PHI_CONTROL = 0,
              PHI_AND_THETA_CONTROL = 1,
              QUICK_POSITION_CONTROL = 2;
  private:
    int POWER;
    int default_pwm;
    BalanbotEncoder mEncoder; 
    Differentiator mDifferentiator;
    double mDirectionCoefficient;
    int mPwmPin, mDirectionPinA, mDirectionPinB, mStandbyPin;
    int mControlMode;
    double mSpeed;
    double mAngle;
    void UpdateAngle();
    void UpdateSpeed();
    void UpdateControl();
  public:  
    BalanbotMotor();
    inline void SetPWMPin(const int pin);
    inline void SetDirectionPins( const int pinA, 
                                  const int pinB );
    inline void SetStandbyPin(const int pin);
    void SetDefaultPWM(int d);
    void SetMotorPins( const int pinPWM, 
                       const int directionPinA, 
                       const int directionPinB, 
                       const int standbyPin);
    void SetEncoderPins(const int interruptPin, 
                        const int directionPin);
    void SetControl(int mode, double reference, double kp, double ki, double kd,double theta_reference=0,double theta_kp=0,double theta_ki=0,double theta_kd=0);
    void InverseRotationDirectionDefinition(const bool ifInverse);
    int GetEncoderInterruptPin();
    double GetSpeed();
    double GetAngle();
    void GetEffort(double &pEffort ,double &iEffort ,double &dEffort,double &effort,double &wpEffort ,double &wiEffort ,double &wdEffort,double &weffort,int &power);
    void Rotate(int voltage);
    void Brake();
    void UpdateEncoder();
    void Update(double phi,double theta);
    void UpdateControl(double phi,double theta);
    
    int GetWheelAngle();
    void move(int speed, int direction);
    
    void clears();

    PIDController angleController;
    PIDController speedController;
};



#endif /* BALANBOTMOTOR_H */
