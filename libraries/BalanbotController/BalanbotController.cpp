#include "BalanbotController.h"

PIDController::PIDController() :
	mKp (0),
	mKi (0),
	mKd (0),
	mReference (0)
{

}

void PIDController::SetPID(float kp, float ki, float kd)
{
	mKp = kp;
	mKi = ki;
	mKd = kd;
}

void PIDController::SetReference(float reference)
{
	mReference = reference;
}

void PIDController::GetIfSteady()
{
	/*if(mError < ERROR_TOLERANCE)
		return true;
	return false;*/
}

float PIDController::Update(float feedback)
{
	float pEffort = 0.0;
	float iEffort = 0.0;
	float dEffort = 0.0;
	float effort = 0.0;
	mError = mReference - feedback;
	//TODO

 	return effort;
}