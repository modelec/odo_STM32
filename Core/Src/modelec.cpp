
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
	int32_t count = __HAL_TIM_GET_COUNTER(&htim3);
	int32_t diff = count - prevCountRight;
	prevCountRight = count;
	float revs = static_cast<float>(diff) / ENCODER_RES;
	return (2*M_PI*WHEEL_RADIUS*revs); // m
}

float DiffBot::readEncoderRight() {
    int32_t count = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t diff = count - prevCountLeft;
    prevCountLeft = count;
    float revs = static_cast<float>(diff) / ENCODER_RES;
    return (2*M_PI*WHEEL_RADIUS*revs); // m
}

void DiffBot::setup() {
	pidLeft = PID(0.2, 0.0, 0.01, -PWM_MAX, PWM_MAX);
	pidRight = PID(0.2, 0.0, 0.01, -PWM_MAX, PWM_MAX);
	pidPos = PID(0.8, 0.0, 0.01, -2, 2);
	pidTheta = PID(0.5, 0.0, 0.01, -M_PI_2, M_PI_2);
}

void DiffBot::stop(bool stop) {
	odo_active = !stop;
	motor.stop(stop);
}

void DiffBot::update(float dt) {
	// read encoder
    float leftVel  = readEncoderLeft();
    float rightVel = readEncoderRight();

    // update pos
    float v = (rightVel + leftVel) / 2.0f;
    float w = (rightVel - leftVel) / WHEEL_BASE;
    pose.x     += v * cosf(pose.theta);
    pose.y     += v * sinf(pose.theta);
    pose.theta += w;

    // pid setup
    float dx = targets[index].x - pose.x;
    float dy = targets[index].y - pose.y;
    float distError = sqrtf(dx*dx + dy*dy);
    float angleTarget = atan2f(dy, dx);
    float angleError  = angleTarget - pose.theta;

    while (angleError >  M_PI) angleError -= 2*M_PI;
    while (angleError < -M_PI) angleError += 2*M_PI;

    switch (targets[index].state) {
    case StatePoint::FINAL:

    	if (fabs(dx) < 0.005 && fabs(dy) < 0.005 && fabs(angleError) < 0.08 /* 5deg */) {
    		stop(true);

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));

    		return;
    	}

    	break;
    case StatePoint::INTERMEDIAIRE:

    	if (fabs(dx) < 0.05 && fabs(dy) < 0.05) {
    		index++;

    		dx = targets[index].x - pose.x;
    		dy = targets[index].y - pose.y;
    	    distError = sqrtf(dx*dx + dy*dy);
    	    angleTarget = atan2f(dy, dx);
    	    angleError  = angleTarget - pose.theta;

    	    while (angleError >  M_PI) angleError -= 2*M_PI;
    	    while (angleError < -M_PI) angleError += 2*M_PI;

    		char log[32];
    		sprintf(log, "SET;WAYPOINT;%d\n", index);
    		CDC_Transmit_FS((uint8_t*)log, strlen(log));
    	}

    	break;
    default:
    	break;
    }

    float vRef = pidPos.compute(0.0, -distError);
    float wRef = pidTheta.compute(targets[index].theta, pose.theta) + 2.0f * angleError;

    float vLeft  = vRef - (WHEEL_BASE_2) * wRef;
    float vRight = vRef + (WHEEL_BASE_2) * wRef;

    float pwmLeft  = pidLeft.compute(vLeft,  leftVel);
    float pwmRight = pidRight.compute(vRight, rightVel);

    motor.leftTarget_PWM  = static_cast<int16_t>(pwmLeft);
    motor.rightTarget_PWM = static_cast<int16_t>(pwmRight);
    motor.update();
}

DiffBot::DiffBot(Point pose, float dt) : pose(pose), dt(dt) {

};

void DiffBot::addTarget(int id, int type, float x, float y, float theta) {

	targets[id] = Point(id, static_cast<StatePoint>(type), x, y, theta);

	if (id <= index) index = 0;

    arrive = false;
}
