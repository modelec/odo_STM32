#ifndef COMM_CALLBACKS_HPP
#define COMM_CALLBACKS_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Déclarations des fonctions appelées depuis le C
void Comm_GetPos(float& x, float& y, float& t);
void Comm_SetPos(float x, float y, float t);

void Comm_GetSpeed(float& vx, float& vy, float& omega);

void Comm_GetPID(float& p, float& i, float& d);
void Comm_SetPID(float p, float i, float d);

void Comm_StartOdometry(bool start);

void Comm_AddWaypoint(int id, int type, float x, float y, float t);

float Comm_GetDistance(int sensorId);

#ifdef __cplusplus
}
#endif

#endif // COMM_CALLBACKS_HPP
