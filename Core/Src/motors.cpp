#include <motors.h>
#include <cstring>
#include <cstdio>
#include "usbd_cdc_if.h"

Motor::Motor(TIM_TypeDef *timer) :
		tim(timer), rightCurrentSpeed(0),leftCurrentSpeed(0), rightTargetSpeed(0),leftTargetSpeed(0), isAccelerating(false), isReversing(
				false), isTurningRight(false), isTurningLeft(false) {
}

void Motor::accelerer(int speed) {
	speed = (speed <= 626) ? speed : 626;
	rightTargetSpeed = speed;
	leftTargetSpeed = speed;
	isAccelerating = true;
	isReversing = false;
	this->stopTurning();
}

void Motor::reculer(int speed) {
	speed = (speed <= 626) ? speed : 626;
	rightTargetSpeed = speed;
	leftTargetSpeed = speed;
	isReversing = true;
	isAccelerating = false;
	this->stopTurning();
}

void Motor::setRightTargetSpeed(int pwm){
	if(pwm < 626){
		this->rightTargetSpeed = pwm;
	}else{
		this->rightTargetSpeed = 626;
	}

}
void Motor::setLeftTargetSpeed(int pwm){
	if(pwm < 626){
		this->leftTargetSpeed = pwm;
	}else{
		this->leftTargetSpeed = 626;
	}

}


void Motor::setLeftCurrentSpeed(float vitesse){
	this->leftCurrentSpeed = vitesse;
}

void Motor::setRightCurrentSpeed(float vitesse){
	this->rightCurrentSpeed = vitesse;
}
float Motor::getRightCurrentSpeed(){
	return this->rightCurrentSpeed;
}
float Motor::getLeftCurrentSpeed(){
	return this->leftCurrentSpeed;
}

void Motor::setRightCurrentPWM(int32_t pwm) {
    this->rightCurrent_PWM = pwm;
}

void Motor::setLeftCurrentPWM(int32_t pwm) {
    this->leftCurrent_PWM = pwm;
}

void Motor::setRightTargetPWM(int32_t pwm) {
    this->rightTarget_PWM = pwm;
}

void Motor::setLeftTargetPWM(int32_t pwm) {
    this->leftTarget_PWM = pwm;
}

int32_t Motor::getRightCurrentPWM() const {
    return this->rightCurrent_PWM;
}

int32_t Motor::getLeftCurrentPWM() const {
    return this->leftCurrent_PWM;
}

int32_t Motor::getRightTargetPWM() const {
    return this->rightTarget_PWM;
}

int32_t Motor::getLeftTargetPWM() const {
    return this->leftTarget_PWM;
}


void Motor::stop() {
	rightTargetSpeed = 0;
	leftTargetSpeed = 0;
	this->stopTurning();
}

bool Motor::isStopped() {
	return rightCurrentSpeed == 0 && leftCurrentSpeed == 0 && rightTargetSpeed == 0 && leftTargetSpeed == 0;
}

void Motor::tournerDroite(int speed) {
	speed = (speed <= 626) ? speed : 626;
	rightTargetSpeed = speed / 2;
	leftTargetSpeed = speed;
	isTurningRight = true;
	isTurningLeft = false;
	isAccelerating = true;
}

void Motor::tournerGauche(int speed) {
	speed = (speed <= 626) ? speed : 626;
	rightTargetSpeed = speed;
	leftTargetSpeed = speed / 2;
	isTurningRight = false;
	isTurningLeft = true;
	isAccelerating = true;
}
void Motor::stopTurning() {
	isTurningLeft = false;
	isTurningRight = false;
}

void Motor::update() {

	// Appliquer targetSpeed dans currentSpeed
    this->leftCurrent_PWM = this->leftTarget_PWM;
    this->rightCurrent_PWM = this->rightTarget_PWM;

    // Contrôle moteur A - TIM8 CH1/CH2
        if (this->leftCurrent_PWM >= 0) {
            TIM8->CCR1 = static_cast<uint16_t>(this->leftCurrent_PWM);  // IN1A (avant)
            TIM8->CCR2 = 0;                                              // IN2A
        } else {
            TIM8->CCR2 = static_cast<uint16_t>(-this->leftCurrent_PWM); // IN2A (arrière)
            TIM8->CCR1 = 0;                                              // IN1A
        }

        // Contrôle moteur B - TIM1 CH1/CH2
        if (this->rightCurrent_PWM >= 0) {
            TIM1->CCR1 = static_cast<uint16_t>(this->rightCurrent_PWM); // IN1B (avant)
            TIM1->CCR2 = 0;                                              // IN2B
        } else {
            TIM1->CCR2 = static_cast<uint16_t>(-this->rightCurrent_PWM); // IN2B (arrière)
            TIM1->CCR1 = 0;                                               // IN1B
        }

    char msg[128];
        snprintf(msg, sizeof(msg),
                 "TIM8->CCR1: %lu | TIM8->CCR2: %lu | TIM1->CCR1: %lu | TIM1->CCR2: %lu\r\n",
                 (uint32_t)TIM8->CCR1, (uint32_t)TIM8->CCR2,
                 (uint32_t)TIM1->CCR1, (uint32_t)TIM1->CCR2);
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

