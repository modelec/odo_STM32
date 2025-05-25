#ifndef COMM_CALLBACKS_HPP
#define COMM_CALLBACKS_HPP

#ifdef __cplusplus
extern "C" {
#endif

// Déclarations des fonctions appelées depuis le C
void Comm_GetPosition(float* x, float* y, float* t);
void Comm_GetSpeed(float* vx, float* vy, float* omega);
void Comm_GetPID(float* p, float* i, float* d);
float Comm_GetDistance(int sensorId); // 👈 AJOUT ICI
void Comm_SetPosition(float x, float y, float t);
void Comm_SetPID(float p, float i, float d);
void Comm_AddWaypoint(int id, int type, float x, float y, float t);
void Comm_StartOdometry(bool start);

#ifdef __cplusplus
}
#endif

#endif // COMM_CALLBACKS_HPP
