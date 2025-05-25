/*
 * CommCallbacks.cpp
 *
 *  Created on: May 25, 2025
 *      Author: maxch
 */



#include "CommCallbacks.hpp"
#include "main.h"
#include "modelec.cpp"
#include "motors.h"
#include "pid.h"
#include "pidPosition.h"

void Comm_GetPosition(float* x, float* y, float* theta) {
    *x = currentPoint.getX();
    *y = currentPoint.getY();
    *theta = currentPoint.getTheta();
}

void Comm_SetPosition(float x, float y, float theta) {
    currentPoint.setX(x);
    currentPoint.setY(y);
    currentPoint.setTheta(theta);
}

void Comm_GetSpeed(float* vx, float* vy, float* omega){
    *vx = vitesseLeft;
    *vy = vitesseRight;
    *omega = vitesseAngulaire;
}

void Comm_GetPID(float* p, float* i, float* d) {
    PIDController::getInstance().getCoefficients(*p, *i, *d);
}

void Comm_SetPID(float p, float i, float d) {
    PIDController::getInstance().setCoefficients(p, i, d);
}

void Comm_StartOdometry(bool on) {
    if (on) Odometry::getInstance().start();
    else Odometry::getInstance().stop();
}

void Comm_AddWaypoint(int id, int type, float x, float y, float theta) {
    Waypoint wp(id, type, x, y, theta);
    WaypointManager::getInstance().addWaypoint(wp);
}

