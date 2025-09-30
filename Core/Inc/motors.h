#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

float approach(float current, float target, float step);

class Motor {
public:
    float leftTarget_PWM = 0, rightTarget_PWM = 0;
    float leftCurrent_PWM = 0, rightCurrent_PWM = 0;
    bool bStop = false;

    void update();
    void stop(bool stop);
};


#endif /* INC_MOTORS_H_ */
