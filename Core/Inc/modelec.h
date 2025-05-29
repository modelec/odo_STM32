/*
 * modelec.h
 *
 *  Created on: May 25, 2025
 *      Author: maxch
 */

#ifndef MODELEC_H
#define MODELEC_H
#include "motors.h"
#include "main.h"
//#include "stm32l0xx_hal.h"
#include "stm32g4xx_hal.h"
//#include "stm32g4xx_hal_uart.h"

#include <cstdio>
#include <cstring>
#include <math.h>
#include <algorithm>
#include "pidVitesse.h"
#include "pid.h"
#include "point.h"
#include "pidPosition.h"
#include "CommCallbacks.h"
#include "usbd_cdc_if.h"
#include "commSTM.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes des fonctions accessibles depuis main.cpp ou ailleurs

void ModelecOdometrySetup(void **out_pid, void **out_pidG, void **out_pidD);
//void ModelecOdometryLoop(void* pid, void* pidG, void* pidD);
void ModelecOdometryLoop(void* pid, void* pidG, void* pidD, int* cnt);
void ModelecOdometryUpdate();

void determinationCoefPosition(
    Point objectifPoint,
    Point pointActuel,
    PidPosition& pid,
    PidVitesse& pidG,
    PidVitesse& pidD,
    float vitGauche,
    float vitDroit,
    int cnt
);

// Fonctions utilitaires (optionnellement utilisables ailleurs)
bool isDelayPassedFrom(uint32_t delay, uint32_t *lastTick);
bool isDelayPassed(uint32_t delay);
void stopMotorsStep();
extern Point targetPoint;


//variables :
extern Point currentPoint;
extern float vitesseLineaire;
extern float vitesseAngulaire;
extern float vitesseLeft;
extern float vitesseRight;
extern bool odo_active;
extern bool arrive;

#ifdef __cplusplus
}
#endif

#endif // MODELEC_H
