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
#include "pidPosition.h"

void Comm_GetPosition(float* x, float* y, float* theta) {
	*x = currentPoint.getX() * 1000.0f;     // convertit 0.7 m en 700.0 mm
	*y = currentPoint.getY() * 1000.0f;
    *theta = currentPoint.getTheta();
}

void Comm_SetPosition(float x, float y, float theta) {
	currentPoint.setX(x / 1000.0f);      // mm → m
	currentPoint.setY(y / 1000.0f);      // mm → m
    currentPoint.setTheta(theta);
}

void Comm_GetSpeed(float* vx, float* vy, float* omega){
	*vx = vitesseLeft * 1000.0f;   // m/s → mm/s
	*vy = vitesseRight * 1000.0f;  // m/s → mm/s
    *omega = vitesseAngulaire;
}

void Comm_GetPID(float* p, float* i, float* d) {
    //PIDController::getInstance().getCoefficients(*p, *i, *d);
}

void Comm_SetPID(float p, float i, float d) {
    //PIDController::getInstance().setCoefficients(p, i, d);
}

void Comm_StartOdometry(bool on) {
	odo_active = on;



}


void Comm_AddWaypoint(int id, int type, float x, float y, float theta) {
	targetPoint.setX(x / 1000.0f);     // conversion mm → m
	targetPoint.setY(y / 1000.0f);
	targetPoint.setTheta(theta);
    arrive = false;
}

float Comm_GetDistance(int n) {
    //return 0.0f; // TODO: remplacer par la bonne logique
}
