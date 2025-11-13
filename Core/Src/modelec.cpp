
#include "modelec.h"

#include <algorithm>
#include <cmath>

bool DiffBot::isDelayPassedFrom(uint32_t delay, uint32_t &lastTick) {
	if (HAL_GetTick() - lastTick >= delay) {
		lastTick = HAL_GetTick();
		return true;
	}
	return false;
}

bool DiffBot::isDelayPassed(uint32_t delay) {
	return isDelayPassedFrom(delay, lastTick);
}

float DiffBot::readEncoderLeft() {
	int16_t count = __HAL_TIM_GET_COUNTER(&htim2);
	int16_t diff = count - prevCountLeft;
	prevCountLeft = count;
	float revs = static_cast<float>(diff) / ENCODER_RES;
	return (2.0f*M_PI*WHEEL_RADIUS*revs); // m
}

float DiffBot::readEncoderRight() {
    int16_t count = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = count - prevCountRight;
    prevCountRight = count;
    float revs = static_cast<float>(diff) / ENCODER_RES;
    return (2.0f*M_PI*WHEEL_RADIUS*revs); // m
}

DiffBot::DiffBot(Point pos, float dt) : pos(pos), dt(dt) {
};

void DiffBot::stop(bool stop) {
	odo_active = !stop;
	motor.stop(stop);
}

void DiffBot::setup() {
	pidLeft = PID(2, 0.0, 0.0, -PWM_MAX, PWM_MAX);
	pidRight = PID(2, 0.0, 0.0, -PWM_MAX, PWM_MAX);
	pidPos = PID(3, 0.0, 0.0, -V_MAX, V_MAX);
	pidTheta = PID(4, 0.0, 0.0, -M_PI, M_PI);

	prevCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
	prevCountRight = __HAL_TIM_GET_COUNTER(&htim3);
}

void DiffBot::update(float dt) {
	if (!isDelayPassed(dt*1000)) return;

	// read encoder
    float rightVel = readEncoderRight();
    float leftVel = readEncoderLeft();

    if (rightVel == 0 && leftVel == 0 && (motor.rightTarget_PWM != 0 || motor.leftTarget_PWM != 0)) {
        // TODO add something when the robot is stuck on the wall so the motor value are >= 0 but the encoder value are = 0
    }

    // update pos
    float v = (leftVel + rightVel) / 2.0f;
    float w = (rightVel - leftVel) / WHEEL_BASE;
    pos.x += v * cosf(pos.theta - w/2);
    pos.y += v * sinf(pos.theta - w/2);
    pos.theta += w;

    while (pos.theta >  M_PI) pos.theta -= 2*M_PI;
    while (pos.theta < -M_PI) pos.theta += 2*M_PI;

    if (odo_active && isDelayPassedFrom(frequencyPublish, publishLastTick)) {
        publishStatus();
    }

    if (!odo_active || !targets[index].active) {
    	motor.update();
    	return;
    }

    // pid setup
    float dx = targets[index].x - pos.x;
    float dy = targets[index].y - pos.y;

    switch (targets[index].state) {
    case FINAL:

    	if (std::fabs(dx) <= precisePosFinal && std::fabs(dy) <= precisePosFinal && std::fabs(targets[index].theta - pos.theta) < preciseAngle) {
    		targets[index].active = false;
    		motor.stop(true);

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));

    		resetPID();

    		return;
    	}

    	break;
    case INTERMEDIAIRE:

    	if (std::fabs(dx) < precisePos && std::fabs(dy) < precisePos) {

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));

    		targets[index].active = false;
    		index++;

    		if (index >= 9) {
    			index = 0;
    			return;
    		}

    		resetPID();

    		dx = targets[index].x - pos.x;
    		dy = targets[index].y - pos.y;

    	}

    	break;
    default:
    	break;
    }

	float dist = sqrtf(dx*dx + dy*dy);

	float angleTarget = atan2f(dy, dx);
	float angleError = angleTarget - pos.theta;

	while (angleError > M_PI) angleError -= 2*M_PI;
	while (angleError < -M_PI) angleError += 2*M_PI;

	float direction = 1.0f;
	if (std::fabs(angleError) > M_PI/2) {
		direction = -1.0f;
		angleError > 0 ? angleError -= M_PI : angleError += M_PI;
	}

	if (std::fabs(angleError) <= precisePosFinal) angleError = 0;

	float distError = dist * cosf(angleError);

	distError *= direction;

    float vRef = pidPos.compute(0.0, -distError, dt);
	float wRef;

    if (targets[index].state == FINAL && std::fabs(dx) <= precisePosFinal && std::fabs(dy) <= precisePosFinal) {
        wRef = pidTheta.compute(targets[index].theta, pos.theta, dt);
        vRef = 0;
    }
    else {
        wRef = pidTheta.compute(0.0, -angleError, dt);
    }

    float vLeft = vRef - (WHEEL_BASE_2) * wRef;
    float vRight = vRef + (WHEEL_BASE_2) * wRef;

    float pwm_ff_left = (vLeft / V_MAX) * PWM_MAX;
    float pwm_ff_right = (vRight / V_MAX) * PWM_MAX;

    float pwm_corr_left = pidLeft.compute(vLeft, leftVel, dt);
    float pwm_corr_right = pidRight.compute(vRight, rightVel, dt);

    float pwmLeft = pwm_ff_left + pwm_corr_left;
    float pwmRight = pwm_ff_right + pwm_corr_right;

    const float pwm_deadzone = 50.0f;
    if (std::fabs(pwmLeft) > 0 && std::fabs(pwmLeft) < pwm_deadzone)
        pwmLeft = (pwmLeft > 0) ? pwm_deadzone : -pwm_deadzone;
    if (std::fabs(pwmRight) > 0 && std::fabs(pwmRight) < pwm_deadzone)
        pwmRight = (pwmRight > 0) ? pwm_deadzone : -pwm_deadzone;

    // saturation
    pwmLeft  = std::max(-PWM_MAX, std::min(pwmLeft,  PWM_MAX));
    pwmRight = std::max(-PWM_MAX, std::min(pwmRight, PWM_MAX));

    motor.leftTarget_PWM = static_cast<int16_t>(pwmLeft);
    motor.rightTarget_PWM = static_cast<int16_t>(pwmRight);
    motor.update();
}

void DiffBot::addTarget(int id, int type, float x, float y, float theta) {

	if (id >= MAX_WAYPOINTS) return;

	targets[id] = Point(id, static_cast<StatePoint>(type), x, y, theta);
	targets[id].active = true;

	if (id <= index) index = 0;

    arrive = false;
}

void DiffBot::resetPID() {
	pidLeft.reset();
	pidRight.reset();
	pidPos.reset();
	pidTheta.reset();
}

void DiffBot::publishStatus() {
    char response[64];
    snprintf(response, sizeof(response), "SET;POS;%.4f;%.4f;%.4f\n", pos.x*1000, pos.y*1000, pos.theta);
    CDC_Transmit_FS((uint8_t*)response, strlen(response));
}
