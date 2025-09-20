#include <motors.h>
#include <cstring>
#include <cstdio>
#include "usbd_cdc_if.h"

void Motor::update() {

	if (bStop) return;

	int16_t PWM_MAX_M = 626;

	if (leftTarget_PWM > PWM_MAX_M) leftTarget_PWM = PWM_MAX_M;
	if (leftTarget_PWM < -PWM_MAX_M) leftTarget_PWM = -PWM_MAX_M;
	if (rightTarget_PWM > PWM_MAX_M) rightTarget_PWM = PWM_MAX_M;
	if (rightTarget_PWM < -PWM_MAX_M) rightTarget_PWM = -PWM_MAX_M;

	leftCurrent_PWM  = leftTarget_PWM;
	rightCurrent_PWM = rightTarget_PWM;

	// moteur gauche -> TIM8 CH1/CH2
	if (leftCurrent_PWM >= 0) {
		TIM8->CCR1 = static_cast<uint16_t>(leftCurrent_PWM);
		TIM8->CCR2 = 0;
	} else {
		TIM8->CCR2 = static_cast<uint16_t>(-leftCurrent_PWM);
		TIM8->CCR1 = 0;
	}

	// moteur droit -> TIM1 CH1/CH2
	if (rightCurrent_PWM >= 0) {
		TIM1->CCR1 = static_cast<uint16_t>(rightCurrent_PWM);
		TIM1->CCR2 = 0;
	} else {
		TIM1->CCR2 = static_cast<uint16_t>(-rightCurrent_PWM);
		TIM1->CCR1 = 0;
	}
}

void Motor::stop(bool stop) {
	bStop = stop;

	if (stop) {
		TIM1->CCR2 = 0;
		TIM1->CCR1 = 0;

		TIM8->CCR1 = 0;
		TIM8->CCR2 = 0;
	}
}
