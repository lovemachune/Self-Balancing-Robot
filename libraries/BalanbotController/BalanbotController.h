#ifndef BALANBOTCONTROLLER_H
#define BALANBOTCONTROLLER_H

#include <NumericalTool.h>
class PIDController{
	const float ERROR_TOLERANCE = 0.5;

	private:
		bool mSteady;
		float mReference;
		float mKp, mKi, mKd;
		float mError;
		Differentiator mDifferentiator;
    	Integrator mIntegrator;  

	public: 
		PIDController();
		void SetPID(float kp, float ki, float kd);
		void SetReference(float reference);
		bool GetIfSteady();
		float Update(float feedback);
};

#endif //CONTROLLER_H