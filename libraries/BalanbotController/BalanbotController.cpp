#include "BalanbotController.h"

PIDController::PIDController() :
	mKp (0),
	mKi (0),
	mKd (0),
	mReference (0)
{

}

void PIDController::SetPID(double kp, double ki, double kd)
{
	mKp = kp;
	mKi = ki;
	mKd = kd;
}

void PIDController::SetReference(double reference)
{
	mReference = reference;
}

bool PIDController::GetIfSteady()
{
	if(mError < ERROR_TOLERANCE)
		return true;
	return false;
}

double PIDController::Update(double feedback)
{
	double pEffort = 0.0;
	double iEffort = 0.0;
	double dEffort = 0.0;
	double effort = 0.0;
	mError = mReference - feedback;
	//TODO
	pEffort = mKp * mError;
	iEffort = mKi * mIntegrator.integral(mError);
	dEffort = mKd * mDifferentiator.differential(mError);
	effort = pEffort + iEffort + dEffort;
	
	//for debug
	PEffort = pEffort;
	IEffort = iEffort;
	DEffort = dEffort;
	Effort = effort;
	return effort;
}

void PIDController::GetEffort(double &pEffort ,double &iEffort ,double &dEffort,double &effort)
{
  pEffort = PEffort;
  iEffort = IEffort;
  dEffort = DEffort;
  effort = Effort;
}
double PIDController::GetReference()
{
	return mReference;
}

void PIDController::clear()
{
	mIntegrator.initialStates();
	mDifferentiator.initialStates();
}