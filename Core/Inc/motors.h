#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//#include "stm32l0xx_hal.h"
#include "stm32g4xx_hal.h"

class Motor {
public:
    int16_t leftTarget_PWM = 0, rightTarget_PWM = 0;
    int16_t leftCurrent_PWM = 0, rightCurrent_PWM = 0;
    bool bStop;

    void update();
    void stop(bool stop);
};


#endif /* INC_MOTORS_H_ */
