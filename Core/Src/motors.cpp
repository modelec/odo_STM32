#include <motors.h>

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
int16_t Motor::getRightCurrentSpeed(){
	return this->rightCurrentSpeed;
}
int16_t Motor::getLeftCurrentSpeed(){
	return this->leftCurrentSpeed;
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
    this->leftCurrentSpeed = this->leftTargetSpeed;
    this->rightCurrentSpeed = this->rightTargetSpeed;

    // Contrôle moteur A - TIM8 CH1/CH2
    if (this->leftCurrentSpeed >= 0) {
        TIM8->CCR1 = static_cast<uint16_t>(this->leftCurrentSpeed);  // IN1A (avant)
        TIM8->CCR2 = 0;                                               // IN2A
    } else {
        TIM8->CCR2 = static_cast<uint16_t>(-this->leftCurrentSpeed); // IN2A (arrière)
        TIM8->CCR1 = 0;                                               // IN1A
    }

    // Contrôle moteur B - TIM1 CH1/CH2
    if (this->rightCurrentSpeed >= 0) {
        TIM1->CCR1 = static_cast<uint16_t>(this->rightCurrentSpeed); // IN1B (avant)
        TIM1->CCR2 = 0;                                               // IN2B
    } else {
        TIM1->CCR2 = static_cast<uint16_t>(-this->rightCurrentSpeed); // IN2B (arrière)
        TIM1->CCR1 = 0;                                                // IN1B
    }
}

