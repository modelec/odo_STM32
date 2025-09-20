/*
 * modelec.h
 *
 *  Created on: May 25, 2025
 *      Author: Modelec
 */

#ifndef MODELEC_H
#define MODELEC_H
#include "motors.h"
#include "main.h"
#include "stm32g4xx_hal.h"

#include <cstdio>
#include <cstring>
#include <math.h>
#include <algorithm>
#include "pid.h"
#include "point.h"
#include "CommCallbacks.h"
#include "usbd_cdc_if.h"
#include "commSTM.h"

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

class DiffBot {
public:
	Point target;
	Point targets[10];
	uint8_t index = 0;
	Point pose;

    Motor motor;

    float dt;

    PID pidLeft, pidRight;
    PID pidPos, pidTheta;

    int32_t prevCountLeft = 0;
    int32_t prevCountRight = 0;

	bool odo_active = false;
	bool arrive = false;

	int16_t PWM_MAX = 626;
	float ENCODER_RES = 2048;
	float WHEEL_RADIUS = 0.0405f;
	float WHEEL_BASE = 0.287f;
	float WHEEL_BASE_2 = 0.1435f;

	float readEncoderRight();

	float readEncoderLeft();

	DiffBot(Point pose, float dt);

	void stop(bool stop);

	void setup();

	void setTarget(Point new_target);

    void update(float dt);

    void addTarget(int id, int type, float x, float y, float theta);

};

#ifdef __cplusplus
}
#endif

#endif // MODELEC_H
