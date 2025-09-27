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
#include <cmath>
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
	Point pose;

	Point targets[10];
	uint8_t index = 0;

    Motor motor;

    float dt;

    PID pidLeft, pidRight;
    PID pidPos, pidTheta;

    int16_t prevCountRight = 0, prevCountLeft = 0;

	bool odo_active = false;
	bool arrive = false;

	int16_t PWM_MAX = 626;
	float ENCODER_RES = 2400.0f;
	float WHEEL_DIAMETER = 0.081f;
	float WHEEL_RADIUS = 0.0405f;
	float WHEEL_BASE = 0.287f;
	float WHEEL_BASE_2 = 0.1435f;

	uint32_t lastTick = 0;

	static bool isDelayPassedFrom(uint32_t delay, uint32_t& lastTick);

	bool isDelayPassed(uint32_t delay);

	float readEncoderRight();

	float readEncoderLeft();

	DiffBot(Point pose, float dt);

	void stop(bool stop);

	void setup();

    void update(float dt);

    void addTarget(int id, int type, float x, float y, float theta);

};

#ifdef __cplusplus
}
#endif

#endif // MODELEC_H
