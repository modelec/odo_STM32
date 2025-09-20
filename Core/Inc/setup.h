/*
 * setup.h
 *
 *  Created on: Sep 18, 2025
 *      Author: Modelec
 */

#ifndef INC_SETUP_H_
#define INC_SETUP_H_

#include "modelec.h"

#ifdef __cplusplus
extern "C" {
#endif

void ModelecOdometrySetup();
void ModelecOdometryLoop(float dt);

#ifdef __cplusplus
}
#endif

#endif /* INC_SETUP_H_ */
