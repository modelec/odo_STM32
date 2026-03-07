/*
 * setup.cpp
 *
 *  Created on: Sep 18, 2025
 *      Author: Modelec
 */

#include <setup.h>
#include <modelec.h>
#include "commSTM.h"

DiffBot bot = DiffBot(Point());

void ModelecOdometrySetup() {
	bot.setup();
}

uint32_t lastTick = 0;

void ModelecOdometryLoop() {
	uint32_t currentTick = HAL_GetTick();
	float actualDt = (currentTick - lastTick) / 1000.0f;

	if (actualDt <= 0.0f) return;

	USB_Comm_Process();
	bot.update(actualDt);

	lastTick = currentTick;
}
