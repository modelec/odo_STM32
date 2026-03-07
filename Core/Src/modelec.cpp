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

DiffBot::DiffBot(Point pos, float dt) : pos(pos), dt(dt)
{
};

void DiffBot::stop(bool stop)
{
    odo_active = !stop;
    motor.stop(stop);
}

void DiffBot::setup()
{
    pidLeft = PID(2.5f, 0.1f, 0.0f, -PWM_MAX, PWM_MAX);
    pidRight = PID(2.5f, 0.1f, 0.0f, -PWM_MAX, PWM_MAX);
    pidPos = PID(3.0f, 0.0f, 0.01f, -V_MAX, V_MAX);
    pidTheta = PID(5.0f, 0.0f, 0.01f, -M_PI, M_PI);

    prevCountLeft = __HAL_TIM_GET_COUNTER(&htim2);
    prevCountRight = __HAL_TIM_GET_COUNTER(&htim3);
}

void DiffBot::update(float dt_actual)
{
    this->dt = dt_actual;

    if (!isDelayPassed(dt * 1000)) return;

    // read encoder
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

    /*if (rightVel == 0 && leftVel == 0) {
        if (motor.rightTarget_PWM != 0 || motor.leftTarget_PWM != 0) {
            if (!no_move) {
                no_move = true;
                publishNotMoved = HAL_GetTick();
            }
            else if (isDelayPassedFrom(notMovedMaxTime, publishNotMoved)) {
                motor.stop(true);

                if (targets[index].active)
                {
                    char log[64];
                    sprintf(log, "SET;WAYPOINT;REACH;%d\n", index);
                    CDC_Transmit_FS((uint8_t*)log, strlen(log));

                    targets[index].active = false;
                }

                no_move = false;

                if (action != 0) {
                    switch (action) {
                    case 1: // LEFT
                        if (cos(pos.theta) > 0) {
                            pos.x = 0.1f; // dist fron back to center
                        }
                        else {
                            pos.x = 0.1f; // dist fron center to front
                        }
                        break;
                    case 2: // TOP
                        if (sin(pos.theta) > 0) {
                            pos.y = 2.0f - 0.1f; // dist fron center to front
                        }
                        else {
                            pos.y = 2.0f - 0.1f; // dist fron back to center
                        }
                        break;
                    case 3: // RIGHT
                        if (cos(pos.theta) > 0) {
                            pos.x = 3.0f - 0.1f; // dist fron center to front
                        }
                        else {
                            pos.x = 3.0f - 0.1f; // dist fron back to center
                        }
                        break;
                    case 4: // BOTTOM
                        if (sin(pos.theta) > 0) {
                            pos.y = 0.1f; // dist fron back to center
                        }
                        else {
                            pos.y = 0.1f; // dist fron center to front
                        }
                        break;
                    default:
                        break;
                    }

                    action = 0;

                    addTarget(0, 1, pos.x, pos.y, pos.theta);
                }
            }
        }
    } else {
        if (no_move) {
            no_move = false;
        }
    }*/

    if (std::abs(rightVel) < 0.001f && std::abs(leftVel) < 0.001f &&
        (std::abs(motor.rightTarget_PWM) > 50 || std::abs(motor.leftTarget_PWM) > 50))
    {
        if (!no_move)
        {
            no_move = true;
            publishNotMoved = HAL_GetTick();
        }
        else if (HAL_GetTick() - publishNotMoved > notMovedMaxTime)
        {
            char log[64];
            int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", index);
            CDC_Transmit_FS((uint8_t*)log, len);

            targets[index].active = false;

            index = (index + 1) % MAX_WAYPOINTS;

            motor.stop(true);
            resetPID();
            return;
        }
    }
    else
    {
        no_move = false;
    }

    if (!odo_active || !targets[index].active) {
        motor.update();
        return;
    }

    float dx = targets[index].x - pos.x;
    float dy = targets[index].y - pos.y;
    float dist = sqrtf(dx*dx + dy*dy);
    float angleTarget = atan2f(dy, dx);
    float angleError = normalizeAngle(angleTarget - pos.theta);

    float direction = 1.0f;
    if (std::fabs(angleError) > (float)M_PI / 2.0f) {
        direction = -1.0f;
        angleError = normalizeAngle(angleError - (float)M_PI);
    }

    float vRef = 0.0f;
    float wRef = 0.0f;

    bool reachedPos = (dist < (targets[index].state == FINAL ? precisePosFinal : precisePos));

    if (reachedPos) {
        bool finalized = true;

        if (targets[index].state == FINAL) {
            float finalAngleErr = normalizeAngle(targets[index].theta - pos.theta);
            if (std::fabs(finalAngleErr) > preciseAngleFinal) {
                wRef = pidTheta.compute(0, -finalAngleErr, dt);
                vRef = 0;
                finalized = false;
            }
        }

        if (finalized) {
            char log[64];
            int len = snprintf(log, sizeof(log), "SET;WAYPOINT;REACH;%d\n", index);
            CDC_Transmit_FS((uint8_t*)log, len);

            targets[index].active = false;

            index = (index + 1) % MAX_WAYPOINTS;

            motor.stop(true);
            resetPID();
            return;
        }
    } else {
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

    motor.leftTarget_PWM = static_cast<int16_t>(applyLimits(pwmLeft));
    motor.rightTarget_PWM = static_cast<int16_t>(applyLimits(pwmRight));
    motor.update();

    if (isDelayPassedFrom(frequencyPublish, publishLastTick)) {
        publishStatus();
    }

    /*
    // pid setup
    float dx = targets[index].x - pos.x;
    float dy = targets[index].y - pos.y;

    switch (targets[index].state)
    {
    case FINAL:

        if (std::fabs(dx) <= precisePosFinal && std::fabs(dy) <= precisePosFinal && std::fabs(
            targets[index].theta - pos.theta) < preciseAngleFinal)
        {
            targets[index].active = false;
            motor.stop(true);

            char log[64];
            sprintf(log, "SET;WAYPOINT;REACH;%d\n", index);
            CDC_Transmit_FS((uint8_t*)log, strlen(log));

            resetPID();

            return;
        }

        break;
    case INTERMEDIAIRE:

        if (std::fabs(dx) < precisePos && std::fabs(dy) < precisePos)
        {
            char log[64];
            sprintf(log, "SET;WAYPOINT;REACH;%d\n", index);
            CDC_Transmit_FS((uint8_t*)log, strlen(log));

            targets[index].active = false;
            index++;

            if (index >= 9)
            {
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

    float dist = sqrtf(dx * dx + dy * dy);

    float angleTarget = atan2f(dy, dx);
    float angleError = angleTarget - pos.theta;

    while (angleError > M_PI) angleError -= 2 * M_PI;
    while (angleError < -M_PI) angleError += 2 * M_PI;

    float direction = 1.0f;
    if (std::fabs(angleError) > M_PI / 2)
    {
        direction = -1.0f;
        angleError > 0 ? angleError -= M_PI : angleError += M_PI;
    }

    float distError = dist * cosf(angleError);

    distError *= direction;

    float vRef = pidPos.compute(0.0, -distError, dt);
    float wRef = 0.0f;

    if (targets[index].state == FINAL && std::fabs(dx) <= precisePosFinal && std::fabs(dy) <= precisePosFinal)
    {
        wRef = pidTheta.compute(targets[index].theta, pos.theta, dt);
        vRef = 0;
    }
    else if (precisePos2 < std::abs(distError))
    {
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

    // saturation
    pwmLeft = std::max(-PWM_MAX, std::min(pwmLeft, PWM_MAX));
    pwmRight = std::max(-PWM_MAX, std::min(pwmRight, PWM_MAX));

    motor.leftTarget_PWM = static_cast<int16_t>(pwmLeft);
    motor.rightTarget_PWM = static_cast<int16_t>(pwmRight);
    motor.update();*/
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
