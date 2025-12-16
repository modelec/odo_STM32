/*
 * CommCallbacks.cpp
 *
 *  Created on: May 25, 2025
 *      Author: maxch
 */



#include "CommCallbacks.h"
#include "main.h"
#include "modelec.h"
#include "motors.h"
#include "pid.h"
#include <cmath>

extern DiffBot bot;

void Comm_GetPos(float& x, float& y, float& theta) {
	x = bot.pos.x * 1000;
	y = bot.pos.y * 1000;
	theta = bot.pos.theta;
}

void Comm_SetPos(float x, float y, float theta) {
	theta = std::atan2(std::sin(theta), std::cos(theta));

	bot.pos.x = x / 1000;
	bot.pos.y = y / 1000;
	bot.pos.theta = theta;
}

void Comm_GetSpeed(float& vx, float& vy, float& omega) {
	vx = bot.vx;
	vy = bot.vy;
	omega = bot.vtheta;
}

bool Comm_GetPID(char *pid_name, float &p, float &i, float &d, float &v_min, float &v_max) {
    PID* pid = nullptr;

    if (strcmp(pid_name, "LEFT") == 0) pid = &bot.pidLeft;
    else if (strcmp(pid_name, "RIGHT") == 0) pid = &bot.pidRight;
    else if (strcmp(pid_name, "POS") == 0) pid = &bot.pidPos;
    else if (strcmp(pid_name, "THETA") == 0) pid = &bot.pidTheta;
    else return false;

    p = pid->getKp();
    i = pid->getKi();
    d = pid->getKd();
    v_min = pid->getOutMin();
    v_max = pid->getOutMax();

    return true;
}

bool Comm_SetPID(char *pid_name, float p, float i, float d, float out_min, float out_max) {
    PID* pid = nullptr;

    if (strcmp(pid_name, "LEFT") == 0)       pid = &bot.pidLeft;
    else if (strcmp(pid_name, "RIGHT") == 0) pid = &bot.pidRight;
    else if (strcmp(pid_name, "POS") == 0)   pid = &bot.pidPos;
    else if (strcmp(pid_name, "THETA") == 0) pid = &bot.pidTheta;
    else return false;

    pid->setTunings(p, i, d);

    // only set limits if user provided them
    if (!std::isnan(out_min) && !std::isnan(out_max)) {
        pid->setLimits(out_min, out_max);
    }

    return true;
}

void Comm_StartOdometry(bool on) {
	bot.stop(!on);
}

void Comm_AddWaypoint(int id, int type, float x, float y, float theta) {
	theta = std::atan2(std::sin(theta), std::cos(theta));

	bot.addTarget(id, type, x / 1000.0f, y / 1000.0f, theta);
}

float Comm_GetDistance(int n) {
    return 0.0f;
}

void Comm_SetPWM(float left, float right) {
	bot.motor.leftTarget_PWM = left;
	bot.motor.rightTarget_PWM = right;
}


void Comm_GetPWM(float &left, float &right) {
	left = bot.motor.leftCurrent_PWM;
	right = bot.motor.rightCurrent_PWM;
}

void Comm_SetPublishFrequency(uint32_t frequencyPublish) {
	bot.frequencyPublish = frequencyPublish;
}

void Comm_GetPublishFrequency(uint32_t &frequencyPublish) {
	frequencyPublish = bot.frequencyPublish;
}

float Comm_GetPreciseAngle() {
	return bot.preciseAngle;
}

float Comm_GetPrecisePos() {
	return bot.precisePos;
}

float Comm_GetPrecisePosFinal() {
	return bot.precisePosFinal;
}

void Comm_SetPreciseAngle(float d) {
	bot.preciseAngle = d;
}

void Comm_SetPrecisePos(float d) {
	bot.precisePos = d;
}

void Comm_SetPrecisePosFinal(float d) {
	bot.precisePosFinal = d;
}

float Comm_GetNotMoveTime() {
	return bot.notMovedMaxTime;
}

void Comm_SetNotMoveTime(float time) {
	bot.notMovedMaxTime = time;
}

void Comm_SetAction(uint8_t time) {
	bot.action = time;
}

uint8_t Comm_GetAction() {
	return bot.action;
}

void Comm_SetAlignment(uint8_t action) {

	float x = bot.pos.x;
	float y = bot.pos.y;
	float theta = bot.pos.theta;

	switch (action) {
	case 1: // LEFT
		x = 0.0f;
		break;
	case 2: // TOP
		y = 2.0f;
		break;
	case 3: // RIGHT
		x = 3.0f;
		break;
	case 4: // BOTTOM
		y = 0.0f;
		break;
	default:
		break;
	}

	bot.addTarget(0, 1, x, y, theta);

	Comm_SetAction(action);
}

void Comm_GetWaypoint(uint8_t &id, uint8_t &type, float &x, float &y, float &theta, bool &active) {
	type = bot.targets[id].state;
	x = bot.targets[id].x;
	y = bot.targets[id].y;
	theta = bot.targets[id].theta;
	active = bot.targets[id].active;
}

void Comm_GetActiveWaypoint(uint8_t &id, uint8_t &type, float &x, float &y, float &theta, bool &active) {
	id = bot.index;
	Comm_GetWaypoint(id, type, x, y, theta, active);
}
