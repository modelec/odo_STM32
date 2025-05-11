
#include "pid.h"
//#include "stm32l0xx_hal.h"      // <--- D'abord
#include "stm32g4xx_hal.h"
//#include "stm32g4xx_hal_uart.h"
#include "usbd_cdc_if.h"
//#include "stm32l0xx_hal_uart.h" // optionnel, déjà dans le précédent
#include <cstdio>
#include <cstring>

//extern UART_HandleTypeDef huart2;

// Constructeur
Pid::Pid(float kp, float ki, float kd){
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;

	this->Vmax = 1;	//à remplacer par la vraie valeur, calculable à partir de la rpm
	this->Wmax = 1; //pareil, à remplacer par la vraie valeur
	this->L = 0.3; //à remplacer, à mesurer direct entre les deux roues motrices du robot (attention, valeur à mettre en mètre)
}

// Setters
void Pid::setKp(float k){
	this->kp = k;
}

void Pid::setKi(float k){
	this->ki = k;
}

void Pid::setKd(float k){
	this->kp = k;
}

// Getters
float Pid::getKp(){
	return this->kp;
}

float Pid::getKi(){
	return this->ki;
}

float Pid::getKd(){
	return this->kd;
}


//Methodes
int Pid::getPWMCommand(float vitesse){
	constexpr float VITESSE_MAX = 0.643f;  // m/s
	constexpr int PWM_MAX = 626;           // commande PWM max
	constexpr int PWM_MIN = 30;            // zone morte (optionnelle)

	    // Saturation de la vitesse
    if (vitesse > VITESSE_MAX){
    	vitesse = VITESSE_MAX;
    }
    else if (vitesse < -VITESSE_MAX){
    	vitesse = -VITESSE_MAX;
    }

	    // Conversion m/s -> PWM avec conservation du signe
    float pwm = (PWM_MAX * std::abs(vitesse)) / VITESSE_MAX;





	    // Application d'un seuil minimal pour éviter les très faibles commandes
    /*if (pwm < PWM_MIN){
    	pwm = 0;
    }*/
    int32_t pwm_signed = (vitesse >= 0) ? std::floor(pwm) : -std::floor(pwm);

    char msg[64];
    //sprintf(msg, "Vitesse à atteindre: %.3f m/s, équivalent PWM: %d\r\n", vitesse, pwm_signed);
    //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));


	// Renvoi avec le bon signe
    return pwm_signed;
}




