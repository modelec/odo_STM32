#include "pidVitesse.h"
#include <cstring>
#include <cstdio>
#include "usbd_cdc_if.h"


// Constructeur
PidVitesse::PidVitesse(float kp, float ki, float kd, float consigneVitesseFinale) : Pid(kp, ki, kd) {

    this->erreurVitesse = 0;
    this->erreurVitesse_old = 0;

    this->consigneVitesseFinale = consigneVitesseFinale;

    this->derivee = 0;
    this->integral = 0;

    this->nouvelleConsigneVitesse = 0;
}

// Setters
void PidVitesse::setErreurVitesse(float e) {
    this->erreurVitesse = e;
}

void PidVitesse::setErreurVitesse_old(float e_old) {
    this->erreurVitesse_old = e_old;
}

void PidVitesse::setConsigneVitesseFinale(float vf) {
    this->consigneVitesseFinale = vf;
}

void PidVitesse::setNouvelleConsigneVitesse(float v) {
    this->nouvelleConsigneVitesse = v;
}

void PidVitesse::setDerivee(float d){
	this->derivee = d;
}

void PidVitesse::setIntegral(float i){
	this->integral = i;
}

// Getters
float PidVitesse::getErreurVitesse() {
    return this->erreurVitesse;
}

float PidVitesse::getErreurVitesse_old() {
    return this->erreurVitesse_old;
}

float PidVitesse::getConsigneVitesseFinale() {
    return this->consigneVitesseFinale;
}

float PidVitesse::getNouvelleConsigneVitesse() {
    return this->nouvelleConsigneVitesse;
}

float PidVitesse::getDerivee(){
	return this->derivee;
}

float PidVitesse::getIntegral(){
	return this->integral;
}

// Méthodes spécifiques

float PidVitesse::calculErreurVitesse(float vitesseActuelle) {
    return this->consigneVitesseFinale - vitesseActuelle;
}

void PidVitesse::updateErreurVitesse(float vitesseActuelle) {
    this->erreurVitesse_old = this->erreurVitesse;
    this->erreurVitesse = this->calculErreurVitesse(vitesseActuelle);
}

void PidVitesse::updateNouvelleVitesse(float vitesseActuelle) {
	/*
		 La sortie est calculée en combinant les 3 termes :
		 	 - proportionnel : on multiplie kp par l'erreur
		 	 - integral 	 : on multiplie ki par la somme des erreurs passées
		 	 - derivee		 : on multiplie kd par la variation de l'erreur
	 */
	//vitesseActuelle = 0.0;

    // Mise à jour de l'erreur de vitesse
    this->updateErreurVitesse(vitesseActuelle);

    // Calcul du terme dérivé
    this->derivee = this->erreurVitesse - this->erreurVitesse_old;

    // Mise à jour du terme intégral
    this->integral += this->erreurVitesse;

    // Calcul de la nouvelle consigne de vitesse avec PID
    float pid = this->getKp() * this->erreurVitesse +
                                    this->getKi() * this->integral +
                                    this->getKd() * this->derivee;
    this->nouvelleConsigneVitesse = pid;

    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "[PID] Cons: %.3f | Mes: %.3f | Err: %.3f | I: %.3f | D: %.3f | Out: %.3f\r\n",
             this->consigneVitesseFinale,
             vitesseActuelle,
             this->erreurVitesse,
             this->integral,
             this->derivee,
             this->nouvelleConsigneVitesse);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}
