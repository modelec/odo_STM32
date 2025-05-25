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
    // === Paramètres de sécurité / stabilisation ===
    const float zoneMorteErreur     = 0.02f;   // seuil min pour agir sur l'erreur
    const float zoneMorteSortie    = 0.005f;  // seuil min pour agir sur la sortie
    const float integralMax        = 10.0f;   // anti-windup
    const float deriveeMax         = 1.0f;    // limitation de la dérivée

    // Mise à jour de l'erreur de vitesse
    this->updateErreurVitesse(vitesseActuelle);

    // Zone morte sur l’erreur
    if (fabs(this->erreurVitesse) < zoneMorteErreur) {
        this->erreurVitesse = 0.0f;
    }
    if (fabs(this->consigneVitesseFinale) < 0.001f) {
        this->integral = 0.0f;
        this->derivee = 0.0f;
        this->nouvelleConsigneVitesse = 0.0f;
        return;
    }


    // Calcul du terme dérivé (bruit possible)
    this->derivee = this->erreurVitesse - this->erreurVitesse_old;

    // Limitation de la dérivée (anti-pics)
    if (this->derivee > deriveeMax) this->derivee = deriveeMax;
    else if (this->derivee < -deriveeMax) this->derivee = -deriveeMax;

    // Mise à jour du terme intégral
    this->integral += this->erreurVitesse;

    // Anti-windup sur l'intégrale
    if (this->integral > integralMax) this->integral = integralMax;
    else if (this->integral < -integralMax) this->integral = -integralMax;

    // Calcul de la commande PID
    float pid = this->getKp() * this->erreurVitesse +
                this->getKi() * this->integral +
                this->getKd() * this->derivee;

    // Zone morte sur la sortie PID
    if (fabs(pid) < zoneMorteSortie) {
        pid = 0.0f;
        // Optionnel : reset intégrale pour couper net le mouvement
        this->integral = 0.0f;
    }

    // Application de la commande
    this->nouvelleConsigneVitesse = pid;

    // === DEBUG ===
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "[PID] Cons: %.3f | Mes: %.3f | Err: %.3f | I: %.3f | D: %.3f | Out: %.3f\r\n",
             this->consigneVitesseFinale,
             vitesseActuelle,
             this->erreurVitesse,
             this->integral,
             this->derivee,
             this->nouvelleConsigneVitesse);
    // CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}

