/*
 * setup.cpp
 *
 *  Created on: Sep 18, 2025
 *      Author: Modelec
 */

#include <setup.h>
#include <modelec.h>
#include "commSTM.h"

extern TIM_HandleTypeDef htim6;

DiffBot bot = DiffBot(Point());

uint32_t lastUs = 0;

void ModelecOdometrySetup() {
    HAL_TIM_Base_Start(&htim6);

    lastUs = __HAL_TIM_GET_COUNTER(&htim6);

    bot.setup();
}

void ModelecOdometryLoop() {
    uint32_t currentUs = __HAL_TIM_GET_COUNTER(&htim6);
    uint32_t diffUs;

    if (currentUs >= lastUs) {
        diffUs = currentUs - lastUs;
    } else {
        diffUs = (65535 - lastUs) + currentUs + 1;
    }

    float actualDt = diffUs / 1000000.0f;

    if (actualDt <= 0.0001f || actualDt > 0.1f) {
        if (actualDt > 0.1f) lastUs = currentUs;
        return;
    }

    USB_Comm_Process();
    bot.update(actualDt);

    lastUs = currentUs;
}
