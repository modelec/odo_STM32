/*
 * modelec.h
 *
 *  Created on: May 26, 2025
 *      Author: maxch
 */

#ifndef INC_MODELEC_H_
#define INC_MODELEC_H_



#include "point.h"
#include "pidPosition.h"
#include "pidVitesse.h"

#ifdef __cplusplus
extern "C" {
#endif

// Fonctions accessibles
void ModelecOdometrySetup(void **out_pid, void **out_pidG, void **out_pidD);
void ModelecOdometryUpdate();
void ModelecOdometryLoop(void* pid, void* pidG, void* pidD);
void determinationCoefPosition(Point objectifPoint, Point pointActuel, PidPosition& pid, PidVitesse& pidG, PidVitesse& pidD, float vitGauche, float vitDroit);


// Variables globales accessibles
extern float x;
extern float y;
extern float theta;

extern float vitesseLineaire;
extern float vitesseAngulaire;
extern float vitesseLeft;
extern float vitesseRight;

extern Point currentPoint;

#ifdef __cplusplus
}
#endif


#endif /* INC_MODELEC_H_ */
