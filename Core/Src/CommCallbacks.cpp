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
	bot.pose.x = x;
	bot.pose.y = y;
	bot.pose.theta = theta;
}

void Comm_GetSpeed(float& vx, float& vy, float& omega){
}

void Comm_GetPID(float& p, float& i, float& d) {
}

void Comm_SetPID(float p, float i, float d) {
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
