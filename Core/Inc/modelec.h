/*
 * modelec.h
 *
 *  Created on: May 25, 2025
 *      Author: Modelec
 */

#ifndef MODELEC_H
#define MODELEC_H

#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

#include "motors.h"
#include "pid.h"
#include "point.h"

#define PWM_MAX 640.0f
#define V_MAX 0.643f // m/s
#define A_MAX 10.0f // m/s^2 huge value cause idk what to put there

#define MAX_WAYPOINTS 16
#define ENCODER_RES 2400.0f
#define WHEEL_DIAMETER 0.0815f
#define WHEEL_RADIUS (WHEEL_DIAMETER/2.0f)
#define WHEEL_BASE 0.29f
#define WHEEL_BASE_2 (WHEEL_BASE/2.0f)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

class DiffBot {
public:
	Point pos;

	Point targets[MAX_WAYPOINTS] = {
			Point(0, FINAL, 0, 0, 0),
	};

	uint8_t index = 0;

    Motor motor;

	float vx = 0, vy = 0, vtheta = 0;

    float dt;

    PID pidLeft, pidRight;
    PID pidPos, pidTheta;

    int16_t prevCountRight = 0, prevCountLeft = 0;

	bool odo_active = false;
	bool arrive = false;

	uint32_t publishNotMoved = 0;
	uint32_t notMovedMaxTime = 300;
	bool no_move = false;

	uint8_t action = 0;

	uint32_t lastTick = 0;
	uint32_t publishLastTick = 0;
	uint32_t frequencyPublish = 300;

	float preciseAngleFinal = 0.001f;
	float preciseAngle = 0.39f;
	float precisePosFinal = 0.001f;
	float precisePos2 = 0.02f;
	float precisePos = 0.15f;

	float currentV = 0;
	const float maxAccel = A_MAX;

	static bool isDelayPassedFrom(uint32_t delay, uint32_t& lastTick);

	bool isDelayPassed(uint32_t delay);

	float readEncoderRight();

	float readEncoderLeft();

	DiffBot(Point pos);

	void stop(bool stop);

	void setup();

    void update(float dt_actual);

    void addTarget(int id, int type, float x, float y, float theta);

	void resetPID();

	void publishStatus();

	void handleStallCondition();

	void notifyWaypointReached(int reachedIndex);
};

#endif // MODELEC_H
