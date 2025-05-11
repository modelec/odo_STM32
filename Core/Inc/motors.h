#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//#include "stm32l0xx_hal.h"
#include "stm32g4xx_hal.h"


class Motor {
private:
    TIM_TypeDef *tim;
    float rightCurrentSpeed;
    float leftCurrentSpeed;
    float rightTargetSpeed;
    float leftTargetSpeed;
    bool isAccelerating;
    bool isReversing;
    bool isTurningRight;
    bool isTurningLeft;
    int32_t rightCurrent_PWM;
    int32_t leftCurrent_PWM;
    int32_t rightTarget_PWM;
    int32_t leftTarget_PWM;

public:
    Motor(TIM_TypeDef *timer);
    void setRightTargetSpeed(int pwm);
    void setLeftTargetSpeed(int pwm);
    void setRightCurrentSpeed(float vitesse);
    void setLeftCurrentSpeed(float vitesse);
    float getRightCurrentSpeed();
    float getLeftCurrentSpeed();

    void setRightCurrentPWM(int32_t pwm);
    void setLeftCurrentPWM(int32_t pwm);
    void setRightTargetPWM(int32_t pwm);
    void setLeftTargetPWM(int32_t pwm);

    int32_t getRightCurrentPWM() const;
    int32_t getLeftCurrentPWM() const;
    int32_t getRightTargetPWM() const;
    int32_t getLeftTargetPWM() const;

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
