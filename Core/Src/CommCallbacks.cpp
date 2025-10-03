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

extern DiffBot bot;

void Comm_GetPos(float& x, float& y, float& theta) {
	x = bot.pose.x;
	y = bot.pose.y;
	theta = bot.pose.theta;
}

void Comm_SetPos(float x, float y, float theta) {
	bot.pose.x = x / 1000;
	bot.pose.y = y / 1000;
	bot.pose.theta = theta;
}

void Comm_GetSpeed(float& vx, float& vy, float& omega) {
	vx = bot.vx;
	vy = bot.vy;
	omega = bot.vtheta;
}

bool Comm_GetPID(char *pid, float &p, float &i, float &d) {
	if (strcmp(pid, "LEFT") == 0) {
		p = bot.pidLeft.getKp();
		i = bot.pidLeft.getKi();
		d = bot.pidLeft.getKd();
	}
	else if (strcmp(pid, "RIGHT") == 0) {
		p = bot.pidRight.getKp();
		i = bot.pidRight.getKi();
		d = bot.pidRight.getKd();
	}
	else if (strcmp(pid, "POS") == 0) {
		p = bot.pidPos.getKp();
		i = bot.pidPos.getKi();
		d = bot.pidPos.getKd();
	}
	else if (strcmp(pid, "THETA") == 0) {
		p = bot.pidTheta.getKp();
		i = bot.pidTheta.getKi();
		d = bot.pidTheta.getKd();
	}
	else {
		return false;
	}
	return true;
}

bool Comm_SetPID(char *pid, float p, float i, float d) {
	if (strcmp(pid, "LEFT") == 0) {
		bot.pidLeft.setTunings(p, i, d);
	}
	else if (strcmp(pid, "RIGHT") == 0) {
		bot.pidRight.setTunings(p, i, d);
	}
	else if (strcmp(pid, "POS") == 0) {
		bot.pidPos.setTunings(p, i, d);
	}
	else if (strcmp(pid, "THETA") == 0) {
		bot.pidTheta.setTunings(p, i, d);
	}
	else {
		return false;
	}
	return true;
}

void Comm_StartOdometry(bool on) {
	bot.stop(!on);
}

void Comm_AddWaypoint(int id, int type, float x, float y, float theta) {
	bot.addTarget(id, type, x / 1000.0f, y / 1000.0f, theta);
}

float Comm_GetDistance(int n) {
    return 0.0f;
}

void Comm_SetPWM(float left, float right) {
	bot.motor.leftTarget_PWM = left;
	bot.motor.rightTarget_PWM = right;
}
