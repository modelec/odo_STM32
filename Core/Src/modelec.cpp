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
#include "commSTM.h"

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
void determinationCoefPosition(Point objectifPoint, Point pointActuel, PidPosition& pid, PidVitesse& pidG, PidVitesse& pidD, float vitGauche, float vitDroit){
	//PidPosition pid(0,0,0,0,0,0,objectifPoint);


	pid.setConsignePositionFinale(objectifPoint);
	std::array<double, 2> vitesse = pid.updateNouvelOrdreVitesse(pointActuel, vitGauche, vitDroit);

	char debug_msg[128];
	sprintf(debug_msg, "[CONS] G: %.3f m/s | D: %.3f m/s\r\n", vitesse[0], vitesse[1]);
	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));

	pidG.setConsigneVitesseFinale(vitesse[0]);
	pidD.setConsigneVitesseFinale(vitesse[1]);


	pidG.updateNouvelleVitesse(motor.getLeftCurrentSpeed());
	pidD.updateNouvelleVitesse(motor.getRightCurrentSpeed());



	float nouvelOrdreG = pidG.getNouvelleConsigneVitesse();
	float nouvelOrdreD = pidD.getNouvelleConsigneVitesse();

	sprintf(debug_msg, "[CORR] G: %.3f m/s | D: %.3f m/s\r\n", nouvelOrdreG, nouvelOrdreD);
	CDC_Transmit_FS((uint8_t*)debug_msg, strlen(debug_msg));

	int erreurG = pidG.getPWMCommand(nouvelOrdreG);
	int erreurD = pidD.getPWMCommand(nouvelOrdreD);

	const int maxErreur = 5;

	if (erreurG > maxErreur) {
	    erreurG = maxErreur;
	} else if (erreurG < -maxErreur) {
	    erreurG = -maxErreur;
	}

	if (erreurD > maxErreur) {
	    erreurD = maxErreur;
	} else if (erreurD < -maxErreur) {
	    erreurD = -maxErreur;
	}

	int ordrePWMG = motor.getLeftCurrentPWM() + erreurG;
	int ordrePWMD = motor.getRightCurrentPWM() + erreurD;

	motor.setLeftTargetPWM(ordrePWMG);
	motor.setRightTargetPWM(ordrePWMD);


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

	*out_pid = new PidPosition(
	    0.3,   // kp
	    0.05,  // ki
	    0.5,   // kd
	    0.5,   // Kp_theta
	    0.02,  // Ki_theta
	    0.7,   // Kd_theta
	    Point()
	);
	//*out_pid = new PidPosition(1.2,0.02,0.8,0, 0, 0, Point());
    *out_pidG = new PidVitesse(0.2, 0.05, 0.01, 0);
    *out_pidD = new PidVitesse(0.2, 0.05, 0.01, 0);

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
	float dt = 0.01f; // 10 ms

	// Calcul des vitesses des roues
	float vitesseLeft = distLeft / dt;
	float vitesseRight = distRight / dt;

	// Vitesse linéaire et angulaire du robot
	float vitesseLineaire = (vitesseLeft + vitesseRight) / 2.0f;
	float vitesseAngulaire = (vitesseRight - vitesseLeft) / WHEEL_BASE;

	// Affichage pour debug
	sprintf(msg, "Vitesse G: %.3f m/s | D: %.3f m/s | Lin: %.3f m/s | Ang: %.3f rad/s\r\n",
	        vitesseLeft, vitesseRight, vitesseLineaire, vitesseAngulaire);
	//CDC_Transmit_FS((uint8_t*) msg, strlen(msg));

	//motor.setLeftCurrentSpeed(vitesseLeft);
	//motor.setRightCurrentSpeed(vitesseRight);
	motor.setLeftCurrentSpeed(vitesseLeft);
	motor.setRightCurrentSpeed(vitesseRight);
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
	//GPIOC->ODR ^= (1 << 10);

	//On met à jour toutes les 10ms
	if (isDelayPassed(10)) {
		ModelecOdometryUpdate();
		//USB_Comm_Process();

		//HAL_Delay(1000);
		Point currentPoint(x, y,theta, StatePoint::INTERMEDIAIRE);
		Point targetPoint(0.50, 0.0,0, StatePoint::FINAL);
		char debugMsg[128];
		sprintf(debugMsg, "Speed avant determination : L=%.3f | R=%.3f\r\n",
		motor.getLeftCurrentSpeed(), motor.getRightCurrentSpeed());
		CDC_Transmit_FS((uint8_t*)debugMsg, strlen(debugMsg));


		determinationCoefPosition(targetPoint, currentPoint, *pidPosition, *pidVitesseG, *pidVitesseD, motor.getLeftCurrentSpeed(), motor.getRightCurrentSpeed()); //TODO vérification inversement target et current
		//HAL_Delay(1000);
		motor.update();




	}

	publishStatus();
}

} //extern C end
