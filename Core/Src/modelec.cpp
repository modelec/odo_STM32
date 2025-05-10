#include "motors.h"
#include "main.h"
//#include "stm32l0xx_hal.h"
#include "stm32g4xx_hal.h"
//#include "stm32g4xx_hal_uart.h"
#include <cstdio>
#include <cstring>
#include <math.h>
#include "pidVitesse.h"
#include "pid.h"
#include "point.h"
#include "pidPosition.h"
#include "usbd_cdc_if.h"
extern "C" {

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim21;
//extern UART_HandleTypeDef huart2;

// Variables globales
// Constants
#define COUNTS_PER_REV    2400.0f    // 600 PPR × 4
#define WHEEL_DIAMETER    0.081f       // meters
#define WHEEL_BASE        0.287f       // meters
#define WHEEL_CIRCUMFERENCE (M_PI * WHEEL_DIAMETER)

// Contrôle des moteurs
Motor motor(TIM2);

// Données odométriques
uint16_t lastPosRight, lastPosLeft;
// x et y sont en mètres
float x, y, theta;

uint32_t lastTick = 0;

bool isDelayPassedFrom(uint32_t delay, uint32_t *lastTick) {
	if (HAL_GetTick() - *lastTick >= delay) {
		*lastTick = HAL_GetTick();
		return true;
	}
	return false;
}
bool isDelayPassed(uint32_t delay) {
	return isDelayPassedFrom(delay, &lastTick);
}

//PID
void determinationCoefPosition(Point objectifPoint, Point pointActuel, PidPosition pid, PidVitesse pidG, PidVitesse pidD){
	//PidPosition pid(0,0,0,0,0,0,objectifPoint);


	pid.setConsignePositionFinale(objectifPoint);
	std::array<double, 2> vitesse = pid.updateNouvelOrdreVitesse(pointActuel);
	//PidVitesse pidG(0, 0, 0, vitesse[0]);
	//PidVitesse pidD(0, 0, 0, vitesse[1]);
	pidG.setConsigneVitesseFinale(vitesse[0]);
	pidD.setConsigneVitesseFinale(vitesse[1]);

	pidG.updateNouvelleVitesse(motor.getLeftCurrentSpeed());
	pidD.updateNouvelleVitesse(motor.getRightCurrentSpeed());


	float nouvelOrdreG = pidG.getNouvelleConsigneVitesse();
	float nouvelOrdreD = pidD.getNouvelleConsigneVitesse();

	int ordrePWMG = pidG.getPWMCommand(nouvelOrdreG);
	int ordrePWMD = pidD.getPWMCommand(nouvelOrdreD);


	motor.setLeftTargetSpeed(ordrePWMG);
	motor.setRightTargetSpeed(ordrePWMD);


}
//Odométrie

void ModelecOdometrySetup(void **out_pid, void **out_pidG, void **out_pidD) {
	CDC_Transmit_FS((uint8_t*)"SETUP COMPLETE\n", strlen("SETUP COMPLETE\n"));
	lastPosRight = __HAL_TIM_GET_COUNTER(&htim2);
	lastPosLeft = __HAL_TIM_GET_COUNTER(&htim3);
	x = 0.0f;
	y = 0.0f;
	theta = 0.0f;
	//motor.accelerer(300);

    *out_pid = new PidPosition(0,0,0,0,0,0,Point());
    *out_pidG = new PidVitesse(0, 0, 0, 0);
    *out_pidD = new PidVitesse(0, 0, 0, 0);

	return;

}

void ModelecOdometryUpdate() {
	//On récupère la valeur des compteurs
	uint16_t posRight = __HAL_TIM_GET_COUNTER(&htim2);
	uint16_t posLeft = __HAL_TIM_GET_COUNTER(&htim3);

	//On calcule les deltas
	int16_t deltaLeft = (int16_t) (posLeft - lastPosLeft);
	int16_t deltaRight = (int16_t) (posRight - lastPosRight);

	//On met à jour la dernière position
	lastPosLeft = posLeft;
	lastPosRight = posRight;

	//On convertit en distance (mètres)
	float distLeft = (deltaLeft / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;
	float distRight = (deltaRight / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE;

	//On calcule les déplacements
	float linear = (distLeft + distRight) / 2.0f;
	float deltaTheta = (distRight - distLeft) / WHEEL_BASE;

	//On met à jour la position
	float avgTheta = theta + deltaTheta / 2.0f;
	x += linear * cosf(avgTheta);
	y += linear * sinf(avgTheta);
	theta += deltaTheta;

	//On normalise theta
	theta = fmodf(theta, 2.0f * M_PI);
	if (theta < 0)
		theta += 2.0f * M_PI;

	char msg[128];
	sprintf(msg, " Update current position : X: %.3f m, Y: %.3f m, Theta: %.3f rad\r\n", x, y, theta);
	CDC_Transmit_FS((uint8_t*) msg, strlen(msg));
}

void publishStatus(){

}

void receiveControlParams(){

}

void ModelecOdometryLoop(void* pid, void* pidG, void* pidD) {
	PidPosition* pidPosition = static_cast<PidPosition*>(pid);
	PidVitesse* pidVitesseG = static_cast<PidVitesse*>(pidG);
	PidVitesse* pidVitesseD = static_cast<PidVitesse*>(pidD);

	//receiveControlParams();
	GPIOC->ODR ^= (1 << 10);

	//On met à jour toutes les 10ms
	if (isDelayPassed(10)) {
		ModelecOdometryUpdate();
		Point currentPoint(x, y,theta, StatePoint::INTERMEDIAIRE);
		Point targetPoint(0.20, 0.20,0, StatePoint::FINAL);
		determinationCoefPosition(currentPoint,targetPoint, *pidPosition, *pidVitesseG, *pidVitesseD);

		//motor.update();

	}

	publishStatus();
}

} //extern C end
