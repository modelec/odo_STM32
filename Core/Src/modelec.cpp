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
    pidLeft = PID(4.f, 0.1f, 0.f, -PWM_MAX, PWM_MAX);
    pidRight = PID(4.f, 0.1f, 0.f, -PWM_MAX, PWM_MAX);
    pidPos = PID(6.0f, 0.0f, 0.1f, -V_MAX, V_MAX);
    pidTheta = PID(10.0f, 0.0f, 0.1f, -M_PI, M_PI);

    prevCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
    prevCountRight = __HAL_TIM_GET_COUNTER(&htim3);
}

void DiffBot::update(float dt_actual)
{
    this->dt = dt_actual;

    if (!isDelayPassed(dt * 1000))
    {
        motor.update();
        return;
    }

    float rightVel = readEncoderRight();
    float leftVel = readEncoderLeft();

    float dLeft = leftVel * dt;
    float dRight = rightVel * dt;
    float dDistance = (dLeft + dRight) / 2.0f;
    float dTheta = (dRight - dLeft) / WHEEL_BASE;

    float avgTheta = pos.theta + (dTheta * 0.5f);
    pos.x += dDistance * cosf(avgTheta);
    pos.y += dDistance * sinf(avgTheta);
    pos.theta = normalizeAngle(pos.theta + dTheta);

    if (std::abs(rightVel) == 0.0f && std::abs(leftVel) == 0.0f &&
        (std::abs(motor.rightTarget_PWM) > 0 || std::abs(motor.leftTarget_PWM) > 0))
    {
        if (!no_move)
        {
            no_move = true;
            publishNotMoved = HAL_GetTick();
        }
        else if (HAL_GetTick() - publishNotMoved > notMovedMaxTime)
        {
            /*if (targets[index].active)
            {
                char log[64];
                sprintf(log, "SET;WAYPOINT;REACH;%d\n", index);
                CDC_Transmit_FS((uint8_t*)log, strlen(log));

                targets[index].active = false;
            }

            index = (index + 1) % MAX_WAYPOINTS;

            motor.stop(true);
            resetPID();
            return;*/
        }
    }
    else
    {
        no_move = false;
    }

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

    bool isAtPoint = (dist < (targets[index].state == FINAL ? precisePosFinal : precisePos));

    if (isAtPoint) {
        if (targets[index].state == FINAL) {
            float finalAngleErr = normalizeAngle(targets[index].theta - pos.theta);

            if (std::fabs(finalAngleErr) <= preciseAngleFinal) {
                char log[64];
                int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", index);
                CDC_Transmit_FS((uint8_t*)log, len);

                targets[index].active = false;
                motor.stop(true);
                resetPID();
                return;
            }

            vRef = 0;
            wRef = pidTheta.compute(0, -finalAngleErr, dt);

        } else {
            char log[64];
            int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", index);
            CDC_Transmit_FS((uint8_t*)log, len);

            targets[index].active = false;
            index = (index + 1) % MAX_WAYPOINTS;
            resetPID();
            return;
        }
    } else {
        float angleTarget = atan2f(dy, dx);
        float angleError = normalizeAngle(angleTarget - pos.theta);

        float direction = 1.0f;
        if (std::fabs(angleError) > static_cast<float>(M_PI) / 2.0f) {
            direction = -1.0f;
            angleError = normalizeAngle(angleError - static_cast<float>(M_PI));
        }

        wRef = pidTheta.compute(0, -angleError, dt);

        float alignmentScale = cosf(angleError);
        if (alignmentScale < 0) alignmentScale = 0;

        vRef = pidPos.compute(0, -dist * direction, dt) * alignmentScale;
    }

    float vLeftReq = vRef - (WHEEL_BASE_2 * wRef);
    float vRightReq = vRef + (WHEEL_BASE_2 * wRef);

    float pwmLeft = ((vLeftReq / V_MAX) * PWM_MAX) + pidLeft.compute(vLeftReq, leftVel, dt);
    float pwmRight = ((vRightReq / V_MAX) * PWM_MAX) + pidRight.compute(vRightReq, rightVel, dt);

    auto applyLimits = [&](float pwm) {
        if (std::abs(pwm) < 1.0f) return 0.0f;
        float limited = std::max(-PWM_MAX, std::min(pwm, PWM_MAX));
        return limited;
    };

    auto applyDeadzone = [&](float pwm) {
        const float min_pwm = 35.0f;
        if (std::abs(pwm) < 1.0f) return 0.0f;

        if (pwm > 0) return pwm + min_pwm;
        else return pwm - min_pwm;
    };

    motor.leftTarget_PWM = static_cast<int16_t>(applyLimits(applyDeadzone(pwmLeft)));
    motor.rightTarget_PWM = static_cast<int16_t>(applyLimits(applyDeadzone(pwmRight)));
    motor.update();
}

void DiffBot::addTarget(int id, int type, float x, float y, float theta)
{
    if (id >= MAX_WAYPOINTS) return;

    targets[id] = Point(id, static_cast<StatePoint>(type), x, y, theta);
    targets[id].active = true;

    if (id <= index) index = 0;

    arrive = false;
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
