/*
 * pid.h
 *
 *  Created on: Mar 25, 2025
 *      Author: Modelec
 */

#ifndef INC_PID_H_
#define INC_PID_H_

class PID {
protected:

    float kp, ki, kd;
    float integral = 0.0f;
    float prevError = 0.0f;
    float outMin, outMax;

public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float outMin = 0.0f, float outMax = 0.0f);

    float compute(float setpoint, float measurement, float dt);

    void reset();

    void setTunings(float kp, float ki, float kd) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void setLimits(float out_min, float out_max) {
    	this->outMin = out_min;
    	this->outMax = out_max;
    }

    float getKp() const { return kp; }
    float getKi() const { return ki; }
    float getKd() const { return kd; }
    float getOutMin() const { return outMin; }
    float getOutMax() const { return outMax; }
    void setPrevError(float x);
};


#endif /* INC_PID_H_ */
