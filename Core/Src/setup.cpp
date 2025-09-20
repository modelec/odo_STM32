/*
 * setup.cpp
 *
 *  Created on: Sep 18, 2025
 *      Author: guich
 */

#include <setup.h>


#ifdef __cplusplus
extern "C" {
#endif

DiffBot bot(Point(), 0.01f);

void ModelecOdometrySetup() {
	bot.setup();
}

void ModelecOdometryLoop(float dt) {
	USB_Comm_Process();
	bot.update(dt);
}

#ifdef __cplusplus
} //extern C end
#endif
