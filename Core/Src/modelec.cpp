#include "modelec.h"

#include <algorithm>
#include <cmath>

float normalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

bool DiffBot::isDelayPassedFrom(uint32_t delay, uint32_t& lastTick)
{
    uint32_t currentTick = HAL_GetTick();
    if (currentTick - lastTick >= delay)
    {
        lastTick = currentTick;
        return true;
    }
    return false;
}

bool DiffBot::isDelayPassed(uint32_t delay)
{
    return isDelayPassedFrom(delay, lastTick);
}

float DiffBot::readEncoderLeft()
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = count - prevCountLeft;
    prevCountLeft = count;

    float revs = static_cast<float>(diff) / ENCODER_RES;
    float distance = (2.0f * (float)M_PI * WHEEL_RADIUS * revs);
    return distance / dt;
}

float DiffBot::readEncoderRight()
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = count - prevCountRight;
    prevCountRight = count;

    float revs = static_cast<float>(diff) / ENCODER_RES;
    float distance = (2.0f * (float)M_PI * WHEEL_RADIUS * revs);
    return distance / dt;
}

DiffBot::DiffBot(Point pos) : pos(pos), dt(0)
{
};

void DiffBot::stop(bool stop)
{
    odo_active = !stop;
    motor.stop(stop);
}

void DiffBot::setup()
{
    pidLeft = PID(7.0f, 0.0f, 0.0f, -PWM_MAX, PWM_MAX);
    pidRight = PID(7.0f, 0.0f, 0.0f, -PWM_MAX, PWM_MAX);
    pidPos = PID(6.0f, 0.2f, 0.02f, -V_MAX, V_MAX);
    pidTheta = PID(15.0f, 0.2f, 0.02f, -M_PI, M_PI);

    prevCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
    prevCountRight = __HAL_TIM_GET_COUNTER(&htim3);
}

void DiffBot::handleStallCondition()
{
    motor.stop(true);

    char log[64];
    int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", index);
    CDC_Transmit_FS((uint8_t*)log, len);

    targets[index].active = false;
    no_move = false;
    currentV = 0.0f;

    resetPID();

    index = (index + 1) % MAX_WAYPOINTS;
}

void DiffBot::notifyWaypointReached(int reachedIndex)
{
    char log[64];
    int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", reachedIndex);
    CDC_Transmit_FS((uint8_t*)log, len);

    targets[reachedIndex].active = false;
    targets[reachedIndex].isAtPosition = false;

    resetPID();

    int nextIndex = (reachedIndex + 1) % MAX_WAYPOINTS;
    if (targets[nextIndex].active && targets[reachedIndex].state != FINAL) {
        index = nextIndex;
    } else {
        motor.stop(true);
        currentV = 0.0f;
    }
}

void DiffBot::update(float dt_actual)
{
    if (dt_actual <= 0.0f) return;
    this->dt = dt_actual;

    float vLeftAct  = readEncoderLeft();
    float vRightAct = readEncoderRight();

    float dDistance = ((vLeftAct + vRightAct) * 0.5f) * dt;
    float dTheta = ((vRightAct - vLeftAct) / WHEEL_BASE) * dt;

    float midTheta = pos.theta + (dTheta * 0.5f);
    pos.x += dDistance * cosf(midTheta);
    pos.y += dDistance * sinf(midTheta);
    pos.theta = normalizeAngle(pos.theta + dTheta);

    // bool commandingMove = (std::abs(motor.leftTarget_PWM) > 50 || std::abs(motor.rightTarget_PWM) > 50);
    // bool isStationary = (std::abs(vLeftAct) == 0.0f && std::abs(vRightAct) == 0.0f);

    /*if (commandingMove && isStationary) {
        if (!no_move)
        {
            no_move = true; publishNotMoved = HAL_GetTick();
        }
        else if (HAL_GetTick() - publishNotMoved > notMovedMaxTime) {
            handleStallCondition();
            return;
        }
    } else {
        no_move = false;
    }*/

    if (odo_active && isDelayPassedFrom(frequencyPublish, publishLastTick)) {
        publishStatus();
    }

    if (!odo_active || !targets[index].active) {
        motor.update();
        return;
    }

    float dx = targets[index].x - pos.x;
    float dy = targets[index].y - pos.y;
    float dist = sqrtf(dx*dx + dy*dy);

    float vRef = 0.0f;
    float wRef = 0.0f;

    float arrivalThreshold = (targets[index].state == FINAL) ? precisePosFinal : precisePos;

    if (dist < arrivalThreshold || targets[index].isAtPosition) {
        if (targets[index].state == FINAL) {
            targets[index].isAtPosition = true;
            float angleErr = normalizeAngle(targets[index].theta - pos.theta);

            if (std::abs(angleErr) <= preciseAngleFinal) {
                notifyWaypointReached(index);
                return;
            }
            wRef = pidTheta.compute(0, -angleErr, dt); // Pivot in place
        } else {
            notifyWaypointReached(index);
            return;
        }
    } else {
        float angleTarget = atan2f(dy, dx);
        float angleError  = normalizeAngle(angleTarget - pos.theta);

        float direction = 1.0f;
        if (std::abs(angleError) > (float)M_PI_2) {
            direction = -1.0f;
            angleError = normalizeAngle(angleError - (float)M_PI);
        }

        wRef = pidTheta.compute(0, -angleError, dt);
        float alignScale = std::max(0.0f, cosf(angleError));
        float targetV = pidPos.compute(0, -dist * direction, dt) * alignScale;

        float maxStep = maxAccel * dt;
        currentV = std::clamp(targetV, currentV - maxStep, currentV + maxStep);
        vRef = currentV;
    }

    float vLeftReq  = vRef - (WHEEL_BASE_2 * wRef);
    float vRightReq = vRef + (WHEEL_BASE_2 * wRef);

    auto computeMotorPWM = [&](float vTarget, float vActual, PID& pid) {
        if (std::abs(vTarget) < 0.001f && std::abs(vActual) < 0.001f) return 0.0f;

        float ff = (vTarget / V_MAX) * PWM_MAX;
        float fb = pid.compute(vTarget, vActual, dt);
        float total = ff + fb;

        const float deadzone = 35.0f;
        if (std::abs(total) > 1.0f) {
            total += (total > 0) ? deadzone : -deadzone;
        }
        return std::clamp(total, -PWM_MAX, PWM_MAX);
    };

    motor.leftTarget_PWM  = (int16_t)computeMotorPWM(vLeftReq, vLeftAct, pidLeft);
    motor.rightTarget_PWM = (int16_t)computeMotorPWM(vRightReq, vRightAct, pidRight);

    motor.update();
}

void DiffBot::addTarget(int id, int type, float x, float y, float theta)
{
    if (id >= MAX_WAYPOINTS) return;

    targets[id] = Point(id, static_cast<StatePoint>(type), x, y, theta);
    targets[id].active = true;

    if (id < index) index = 0;

    arrive = false;
    no_move = false;
}

void DiffBot::resetPID()
{
    pidLeft.reset();
    pidRight.reset();
    pidPos.reset();
    pidTheta.reset();
}

void DiffBot::publishStatus()
{
    char response[64];
    snprintf(response, sizeof(response), "SET;POS;%.4f;%.4f;%.4f\n", pos.x * 1000, pos.y * 1000, pos.theta);
    CDC_Transmit_FS((uint8_t*)response, strlen(response));
}
