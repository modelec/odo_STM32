
#include "modelec.h"

// Variables globales
// Constants
// #define COUNTS_PER_REV    2400.0f    // 600 PPR × 4
// #define WHEEL_DIAMETER    0.081f       // meters
// #define WHEEL_BASE        0.287f       // meters
// #define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_DIAMETER)

	/**out_pid = new PidPosition(
		0.8,   // kp — un poil plus agressif, il pousse plus vers la cible
	    0.0,   // ki — toujours off pour éviter du dépassement imprévu
	    0.015, // kd — un peu moins de freinage anticipé

	    0.5,   // kpTheta — peut rester soft pour éviter les oscillations d’orientation
	    0.0,   // kiTheta
	    0.15,  // kdTheta — un peu moins de frein sur la rotation
		2,
	    Point()
	);*/

	// *out_pidG = new PidVitesse(0.2, 0.0, 0.01, 0);
	// *out_pidD = new PidVitesse(0.2, 0.0, 0.01, 0);


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

void DiffBot::setup() {
	pidLeft = PID(1, 0.0, 0.0, -PWM_MAX, PWM_MAX);
	pidRight = PID(1, 0.0, 0.0, -PWM_MAX, PWM_MAX);
	pidPos = PID(1, 0.0, 0.0, -2, 2);
	pidTheta = PID(1, 0.0, 0.0, -M_PI, M_PI);

	prevCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
	prevCountRight = __HAL_TIM_GET_COUNTER(&htim3);
}

void DiffBot::stop(bool stop) {
	odo_active = !stop;
	motor.stop(stop);
}

void DiffBot::update(float dt) {
	if (!isDelayPassed(dt*1000)) return;

	// read encoder
    float rightVel  = readEncoderRight();
    float leftVel = readEncoderLeft();

    // update pos
    float v = (leftVel + rightVel) / 2.0f;
    float w = (rightVel - leftVel) / WHEEL_BASE;
    pose.x     += v * cosf(pose.theta - w/2);
    pose.y     += v * sinf(pose.theta - w/2);
    pose.theta += w;

    while (pose.theta >  M_PI) pose.theta -= 2*M_PI;
    while (pose.theta < -M_PI) pose.theta += 2*M_PI;

    if (!odo_active || !targets[index].active) {
    	motor.update();
    	return;
    }
    /*
     *
     *
     * TODO
     * check les valeurs htim (encodeur) et TIM (moteur)
     *
     *
     * */

    // pid setup
    float dx = targets[index].x - pose.x;
    float dy = targets[index].y - pose.y;

    const float minRes = 0.01f;
    if (fabsf(dx) < minRes) dx = 0;
    if (fabsf(dy) < minRes) dy = 0;

    float dist = sqrtf(dx*dx + dy*dy);

    float angleTarget = atan2f(dy, dx);
    float angleError  = angleTarget - pose.theta;

    while (angleError >  M_PI) angleError -= 2*M_PI;
    while (angleError < -M_PI) angleError += 2*M_PI;

    float direction = 1.0f;
    if (fabs(angleError) > M_PI/2) {
    	direction = -1.0f;
        angleError > 0 ? angleError -= M_PI : angleError += M_PI;
    }

    if (fabs(angleError) <= 0.001) angleError = 0;

    float distError = dist * cosf(angleError);

    distError *= direction;

    switch (targets[index].state) {
    case StatePoint::FINAL:

    	if (fabs(dx) <= 0.01 && fabs(dy) <= 0.01 && fabs(targets[index].theta - pose.theta) < 0.01) {
    		// maybe remove that so when stopped and you moved it, the robot compensates itself
    		targets[index].active = false;
    		motor.stop(true);

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));

    		return;
    	} else if (targets[index].active == false) {
    		targets[index].active = true;
    	}

    	break;
    case StatePoint::INTERMEDIAIRE:

    	if (fabs(dx) < 0.1 && fabs(dy) < 0.1) {

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));

    		targets[index].active = false;
    		index++;

    		if (index >= 9) {
    			index = 0;
    			return;
    		}

    		dx = targets[index].x - pose.x;
    		dy = targets[index].y - pose.y;
    	    distError = sqrtf(dx*dx + dy*dy);
    	    angleTarget = atan2f(dy, dx);
    	    angleError  = angleTarget - pose.theta;

    	    while (angleError > M_PI) angleError -= 2*M_PI;
    	    while (angleError < -M_PI) angleError += 2*M_PI;

    	}

    	break;
    default:
    	break;
    }

    // check if final x and y are close but not theta so only turn

    float vRef = pidPos.compute(0.0, -distError, dt);
    // float wRef = pidTheta.compute(targets[index].theta, pose.theta) /*+ 2.0f * angleError*/;
    float wRef;

    if (targets[index].state == StatePoint::FINAL && fabs(dx) <= 0.01 && fabs(dy) <= 0.01) {
        wRef = pidTheta.compute(targets[index].theta, pose.theta, dt);
        vRef = 0;
    }
    else {
        wRef = pidTheta.compute(0.0, angleError, dt);
    }

    float vLeft  = vRef - (WHEEL_BASE_2) * wRef;
    float vRight = vRef + (WHEEL_BASE_2) * wRef;

    float v_max = 0.643f; // m/s

    float pwm_ff_left  = (vLeft / v_max) * PWM_MAX;
    float pwm_ff_right = (vRight / v_max) * PWM_MAX;

    float pwm_corr_left  = pidLeft.compute(vLeft, leftVel, dt);
    float pwm_corr_right = pidRight.compute(vRight, rightVel, dt);

    float pwmLeft = pwm_ff_left + pwm_corr_left;
    float pwmRight = pwm_ff_right + pwm_corr_right;

    const float pwm_deadzone = 50.0f;
    if (fabs(pwmLeft) > 0 && fabs(pwmLeft) < pwm_deadzone)
        pwmLeft = (pwmLeft > 0) ? pwm_deadzone : -pwm_deadzone;
    if (fabs(pwmRight) > 0 && fabs(pwmRight) < pwm_deadzone)
        pwmRight = (pwmRight > 0) ? pwm_deadzone : -pwm_deadzone;

    // saturation
    pwmLeft  = MAX(-PWM_MAX, MIN(PWM_MAX, pwmLeft));
    pwmRight = MAX(-PWM_MAX, MIN(PWM_MAX, pwmRight));

    motor.leftTarget_PWM  = static_cast<int16_t>(pwmLeft);
    motor.rightTarget_PWM = static_cast<int16_t>(pwmRight);
    motor.update();
}

DiffBot::DiffBot(Point pose, float dt) : pose(pose), dt(dt) {

};

void DiffBot::addTarget(int id, int type, float x, float y, float theta) {

	if (id >= 10) return;

	targets[id] = Point(id, static_cast<StatePoint>(type), x, y, theta);
	targets[id].active = true;

	if (id <= index) index = 0;

    arrive = false;
}
