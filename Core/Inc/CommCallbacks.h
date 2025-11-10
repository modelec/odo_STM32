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

void Comm_SetPublishFrequency(uint32_t frequencyPublish = 20);

void Comm_GetPublishFrequency(uint32_t &frequencyPublish);

#ifdef __cplusplus
}
#endif

#endif // COMM_CALLBACKS_HPP
