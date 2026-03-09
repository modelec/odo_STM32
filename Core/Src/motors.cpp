#include <algorithm>
#include <motors.h>
#include <modelec.h>

float approach(float current, float target, float step) {
	if (current < target)
		return std::min(current + step, target);

	if (current > target)
		return std::max(current - step, target);

	return current;
}

void Motor::update() {

    int16_t max_step = 10;

    leftTarget_PWM  = std::max(-PWM_MAX, std::min(leftTarget_PWM,  PWM_MAX));
    rightTarget_PWM = std::max(-PWM_MAX, std::min(rightTarget_PWM, PWM_MAX));

	leftCurrent_PWM  = approach(leftCurrent_PWM,  leftTarget_PWM,  max_step);
	rightCurrent_PWM = approach(rightCurrent_PWM, rightTarget_PWM, max_step);

	// left motor -> TIM1 CH1/CH2
	if (leftCurrent_PWM >= 0) {
		TIM1->CCR1 = static_cast<uint16_t>(leftCurrent_PWM);
		TIM1->CCR2 = 0;
	} else {
		TIM1->CCR2 = static_cast<uint16_t>(-leftCurrent_PWM);
		TIM1->CCR1 = 0;
	}

	// right motor -> TIM8 CH1/CH2
	if (rightCurrent_PWM >= 0) {
		TIM8->CCR1 = static_cast<uint16_t>(rightCurrent_PWM);
		TIM8->CCR2 = 0;
	} else {
		TIM8->CCR2 = static_cast<uint16_t>(-rightCurrent_PWM);
		TIM8->CCR1 = 0;
	}
}

void Motor::stop(bool stop) {
	bStop = stop;

	if (stop) {
		leftTarget_PWM = 0;
		rightTarget_PWM = 0;
	}
}
