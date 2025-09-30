/*
 * setup.cpp
 *
 *  Created on: Sep 18, 2025
 *      Author: Modelec
 */

#include <setup.h>
#include <modelec.h>
#include "commSTM.h"

DiffBot bot(Point(), 0.01f);

void ModelecOdometrySetup() {
	bot.setup();
}

void ModelecOdometryLoop(float dt) {
	USB_Comm_Process();
	bot.update(dt);
}
