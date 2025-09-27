
#include "pid.h"


PID::PID(float kp, float ki, float kd, float outMin, float outMax)
	: kp(kp), ki(ki), kd(kd), integral(0.0), outMin(outMin), outMax(outMax) {
}

float PID::compute(float setpoint, float measurement, float dt) {
	float error = setpoint - measurement;
	integral += error * dt;
	float derivative = (error - prevError) / dt;
	float output = kp * error + ki * integral + kd * derivative;

	output = std::min(std::max(output, outMin), outMax);

	prevError = error;
	return output;
}
