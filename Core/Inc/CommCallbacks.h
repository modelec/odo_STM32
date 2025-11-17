#ifndef COMM_CALLBACKS_HPP
#define COMM_CALLBACKS_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Déclarations des fonctions appelées depuis le C
void Comm_GetPos(float& x, float& y, float& t);
void Comm_SetPos(float x, float y, float t);

void Comm_GetSpeed(float& vx, float& vy, float& omega);

bool Comm_GetPID(char *pid_name, float &p, float &i, float &d, float &v_min, float &v_max);

bool Comm_SetPID(char *pid_name, float p, float i, float d, float out_min, float out_max);

void Comm_StartOdometry(bool start);

void Comm_AddWaypoint(int id, int type, float x, float y, float t);

float Comm_GetDistance(int sensorId);

void Comm_SetPWM(float left, float right);

void Comm_GetPWM(float &left, float &right);

void Comm_SetPublishFrequency(uint32_t frequencyPublish = 20);

void Comm_GetPublishFrequency(uint32_t &frequencyPublish);

float Comm_GetPreciseAngle();

float Comm_GetPrecisePos();

float Comm_GetPrecisePosFinal();

void Comm_SetPreciseAngle(float d);

void Comm_SetPrecisePos(float d);

void Comm_SetPrecisePosFinal(float d);

float Comm_GetNotMoveTime();

void Comm_SetNotMoveTime(float time);

void Comm_SetAction(uint8_t time);

uint8_t Comm_GetAction();

void Comm_SetAlignment(uint8_t action);

void Comm_GetWaypoint(uint8_t &id, uint8_t &type, float &x, float &y, float &theta, bool &active);

void Comm_GetActiveWaypoint(uint8_t &id, uint8_t &type, float &x, float &y, float &theta, bool &active);

#ifdef __cplusplus
}
#endif

#endif // COMM_CALLBACKS_HPP
