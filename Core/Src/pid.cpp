
#include "pid.h"


PID::PID(float kp, float ki, float kd, float outMin, float outMax)
	: kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax) {
}

float PID::compute(float setpoint, float measurement) {
	float error = setpoint - measurement;
	integral += error;
	float derivative = (error - prevError);
	float output = kp * error + ki * integral + kd * derivative;

	// saturation
	output = std::min(std::max(output, outMin), outMax);

	prevError = error;
	return output;
}
