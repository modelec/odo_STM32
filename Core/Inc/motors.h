#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//#include "stm32l0xx_hal.h"
#include "stm32g4xx_hal.h"


class Motor {
private:
    TIM_TypeDef *tim;
    int16_t rightCurrentSpeed;
    int16_t leftCurrentSpeed;
    int16_t rightTargetSpeed;
    int16_t leftTargetSpeed;
    bool isAccelerating;
    bool isReversing;
    bool isTurningRight;
    bool isTurningLeft;

public:
    Motor(TIM_TypeDef *timer);
    void setRightTargetSpeed(int pwm);
    void setLeftTargetSpeed(int pwm);
    int16_t getRightCurrentSpeed();
    int16_t getLeftCurrentSpeed();
    void accelerer(int speed);
    void reculer(int speed);
    void stop();
    void stopTurning();
    bool isStopped();
    void tournerDroite(int speed);
    void tournerGauche(int speed);
    void update();

};


#endif /* INC_MOTORS_H_ */
