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

void updateA()
{
  motorA.UpdateEncoder();
}
void updateB()
{
  motorB.UpdateEncoder();
}

void setMotor()
{
    motorA.SetMotorPins(PWMA, AIN1, AIN2, STBY);
    motorB.SetMotorPins(PWMB, BIN1, BIN2, STBY);

    motorA.SetEncoderPins(A_Interrupt, A_Direction);
    motorB.SetEncoderPins(B_Interrupt, B_Direction);

    motorA.InverseRotationDirectionDefinition(false);
    motorB.InverseRotationDirectionDefinition(false);

    motorA.SetControl(mode, reference,kp,ki,kd);
    motorB.SetControl(mode, reference,kp,ki,kd);

    attachInterrupt(digitalPinToInterrupt(motorA.GetEncoderInterruptPin()), updateA, RISING);
    attachInterrupt(digitalPinToInterrupt(motorB.GetEncoderInterruptPin()), updateB, RISING);
}