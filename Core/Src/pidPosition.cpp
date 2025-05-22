
#include "pidPosition.h"
#include <cstdio>
#include "usbd_cdc_if.h"
#include "commSTM.h"
PidPosition::PidPosition(float kp, float ki, float kd,float Kp_theta, float Ki_theta, float Kd_theta, Point consignePosition): Pid(kp, ki, kd){

	this->Kp_theta = Kp_theta;
	this->Ki_theta = Ki_theta;
	this->Kd_theta = Kd_theta;

	this->erreurPosition = consignePosition;		// à l'init, la position est à 0,0,0, donc on a consigne - position actuelle qui vaut juste consigne
}





// Setters
void PidPosition::setConsignePositionFinale(Point consigne) {
    this->consignePositionFinale = consigne;
}

void PidPosition::setErreurPosition(Point erreur) {
    this->erreurPosition = erreur;
}

void PidPosition::setErreurPosition_old(Point erreur_old) {
    this->erreurPosition_old = erreur_old;
}

void PidPosition::setD(float d[2]) {
    this->d[0] = d[0];
    this->d[1] = d[1];
}

void PidPosition::setI(float i[2]) {
    this->i[0] = i[0];
    this->i[1] = i[1];
}

void PidPosition::setKpTheta(float kp) {
    this->Kp_theta = kp;
}

void PidPosition::setKiTheta(float ki) {
    this->Ki_theta = ki;
}

void PidPosition::setKdTheta(float kd) {
    this->Kd_theta = kd;
}





// Getters
Point PidPosition::getConsignePositionFinale() {
    return this->consignePositionFinale;
}

Point PidPosition::getErreurPosition() {
    return this->erreurPosition;
}

Point PidPosition::getErreurPosition_old() {
    return this->erreurPosition_old;
}

float PidPosition::getKpTheta() {
    return this->Kp_theta;
}

float PidPosition::getKiTheta() {
    return this->Ki_theta;
}

float PidPosition::getKdTheta() {
    return this->Kd_theta;
}

std::array<float, 2> PidPosition::getD() {
    return {this->d[0], this->d[1]};
}

std::array<float, 2> PidPosition::getI() {
    return {this->i[0], this->i[1]};
}


//Méthodes

// Mise à jour de l'erreur de position
void PidPosition::updateErreurPosition(Point PointActuel) {
    this->erreurPosition_old = erreurPosition;
    this->erreurPosition = calculErreurPosition(PointActuel);
}

// Calcul de l'erreur de position
// Calcul de l'erreur de position
Point PidPosition::calculErreurPosition(Point p) {
    return Point(
        this->consignePositionFinale.getX() - p.getX(),
        this->consignePositionFinale.getY() - p.getY(),
        atan2(sin(this->consignePositionFinale.getTheta() - p.getTheta()),
              cos(this->consignePositionFinale.getTheta() - p.getTheta())),
        StatePoint::NONDETERMINE
    );
}


// Mise à jour et calcul du nouvel ordre de vitesse pour les moteurs
std::array<double, 2> PidPosition::updateNouvelOrdreVitesse(Point pointActuel, float vitGauche, float vitDroit) {
	char log[128];
    this->updateErreurPosition(pointActuel);
    sprintf(log, "[DEBUG] Position Actuelle | X: %.3f | Consigne X: %.3f\r\n", pointActuel.getX(), consignePositionFinale.getX());
    CDC_Transmit_FS((uint8_t*)log, strlen(log));



    // Log de l’erreur de position actuelle
    sprintf(log, "[PID] Erreur Position | X: %.3f | Y: %.3f | Theta: %.3f\r\n",
            erreurPosition.getX(), erreurPosition.getY(), erreurPosition.getTheta());
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Mise à jour du terme intégral
    // Si la commande est saturée, n'accumule pas l'intégrale
    float i1 = this->i[0] + this->erreurPosition.getX();
    float i2 = this->i[1] + this->erreurPosition.getY();
    float i3 = this->i[2] + this->erreurPosition.getTheta();

    sprintf(log, "[PID] Terme Intégral | iX: %.3f | iY: %.3f | iTheta: %.3f\r\n", i[0], i[1], i[2]);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Calcul du terme dérivé
    this->d[0] = this->erreurPosition.getX() - this->erreurPosition_old.getX();
    this->d[1] = this->erreurPosition.getY() - this->erreurPosition_old.getY();
    this->d[2] = this->erreurPosition.getTheta() - this->erreurPosition_old.getTheta();

    sprintf(log, "[PID] Terme Dérivé | dX: %.3f | dY: %.3f | dTheta: %.3f\r\n", d[0], d[1], d[2]);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Calcul des commandes PID
    double commandeX = kp * this->erreurPosition.getX() + ki * i1 + kd * this->d[0];
    double commandeY = kp * this->erreurPosition.getY() + ki * i2 + kd * this->d[1];
    double commandeTheta = Kp_theta * this->erreurPosition.getTheta() + Ki_theta * i3 + Kd_theta * this->d[2];

    sprintf(log, "[PID] Commandes PID | X: %.3f | Y: %.3f | Theta: %.3f\r\n", commandeX, commandeY, commandeTheta);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Conversion en vitesse linéaire et angulaire
    double vitesseLineaire = sqrt(commandeX * commandeX + commandeY * commandeY);
    double vitesseAngulaire = commandeTheta;

    sprintf(log, "[PID] Vitesses Avant Saturation | Linéaire: %.3f m/s | Angulaire: %.3f rad/s\r\n", vitesseLineaire, vitesseAngulaire);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));



    sprintf(log, "[PID] Vitesses Après Saturation | Linéaire: %.3f m/s | Angulaire: %.3f rad/s\r\n", vitesseLineaire, vitesseAngulaire);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));




    // Conversion en vitesse des roues
    double vitesseGauche = vitesseLineaire - (this->L / 2) * vitesseAngulaire;
    double vitesseDroite = vitesseLineaire + (this->L / 2) * vitesseAngulaire;

    if (vitesseGauche < 0.235 && vitesseDroite < 0.235 && vitesseGauche > -0.235 && vitesseDroite > -0.235){
    	this->i[0] = i1;
    	this->i[1] = i2;
    	this->i[2] = i3;
    }

    // Saturation des vitesses des roues
    if (vitesseGauche > 0.235){ vitesseGauche = 0.235;}
    else if (vitesseGauche < -0.235) vitesseGauche = -0.235;

    if (vitesseDroite > 0.235) vitesseDroite = 0.235;
    else if (vitesseDroite < -0.235) vitesseDroite = -0.235;



    sprintf(log, "[SET] VITESSE SORTIE DE PID POS | G: %.3f m/s | D: %.3f m/s\r\n", vitesseGauche, vitesseDroite);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    return {vitesseGauche, vitesseDroite};
    //return {0.235, 0.235};
}

