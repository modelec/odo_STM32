
#include "pid.h"

#include <algorithm>

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
	: kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax) {
	reset();
}

float PID::compute(float setpoint, float measurement, float dt) {
	float error = setpoint - measurement;
	integral += error * dt;
	float derivative = (error - prevError) / dt;
	float output = kp * error + ki * integral + kd * derivative;

	output = std::max(outMin, std::min(output, outMax));

	prevError = error;
	return output;
}

void PID::reset() {
	integral = 0.0f;
	prevError = 0.0f;
}
