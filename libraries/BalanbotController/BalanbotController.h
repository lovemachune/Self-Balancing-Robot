#ifndef BALANBOTCONTROLLER_H
#define BALANBOTCONTROLLER_H

#include <NumericalTool.h>
class PIDController{
	const double ERROR_TOLERANCE = 0.5;

	private:
		bool mSteady;
		double mReference;
		double mKp, mKi, mKd;
		double mError;
		Differentiator mDifferentiator;
    	Integrator mIntegrator;  
		double PEffort ,IEffort ,DEffort,Effort;

	public: 
		PIDController();
		void SetPID(double kp, double ki, double kd);
		void SetReference(double reference);
		bool GetIfSteady();
		void GetEffort(double &pEffort ,double &iEffort ,double &dEffort,double &effort);
		double GetReference();
		double Update(double feedback);
		void clear();
};

#endif //CONTROLLER_H