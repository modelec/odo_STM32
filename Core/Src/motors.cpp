#include <motors.h>
#include <cstring>
#include <cstdio>
#include "usbd_cdc_if.h"

void Motor::update() {

	int16_t PWM_MAX_M = 626;

    uint8_t max_step = 25;

	if (leftTarget_PWM > PWM_MAX_M) leftTarget_PWM = PWM_MAX_M;
	if (leftTarget_PWM < -PWM_MAX_M) leftTarget_PWM = -PWM_MAX_M;
	if (rightTarget_PWM > PWM_MAX_M) rightTarget_PWM = PWM_MAX_M;
	if (rightTarget_PWM < -PWM_MAX_M) rightTarget_PWM = -PWM_MAX_M;

    if (leftCurrent_PWM < leftTarget_PWM) {
        leftCurrent_PWM += max_step;
        if (leftCurrent_PWM > leftTarget_PWM)
            leftCurrent_PWM = leftTarget_PWM;
    } else if (leftCurrent_PWM > leftTarget_PWM) {
        leftCurrent_PWM -= max_step;
        if (leftCurrent_PWM < leftTarget_PWM)
            leftCurrent_PWM = leftTarget_PWM;
    }

    if (rightCurrent_PWM < rightTarget_PWM) {
        rightCurrent_PWM += max_step;
        if (rightCurrent_PWM > rightTarget_PWM)
            rightCurrent_PWM = rightTarget_PWM;
    } else if (rightCurrent_PWM > rightTarget_PWM) {
        rightCurrent_PWM -= max_step;
        if (rightCurrent_PWM < rightTarget_PWM)
            rightCurrent_PWM = rightTarget_PWM;
    }

	// moteur gauche -> TIM1 CH1/CH2
	if (leftCurrent_PWM >= 0) {
		TIM8->CCR1 = static_cast<uint16_t>(leftCurrent_PWM);
		TIM8->CCR2 = 0;
	} else {
		TIM8->CCR2 = static_cast<uint16_t>(-leftCurrent_PWM);
		TIM8->CCR1 = 0;
	}

	// moteur droit -> TIM8 CH1/CH2
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
		leftTarget_PWM = 0;
		rightTarget_PWM = 0;
	}
}
