/*
 * pid.h
 *
 *  Created on: Mar 25, 2025
 *      Author: Modelec
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "point.h"

class PID {
protected:

    float kp, ki, kd;
    float integral = 0.0f;
    float prevError = 0.0f;
    float outMin, outMax;

public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float outMin = 0.0f, float outMax = 0.0f);

    float compute(float setpoint, float measurement);
};


#endif /* INC_PID_H_ */
