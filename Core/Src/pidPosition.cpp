
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

    // Affichage position actuelle vs consigne
    sprintf(log, "[DEBUG] Position Actuelle | X: %.3f | Consigne X: %.3f\r\n", pointActuel.getX(), consignePositionFinale.getX());
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Affichage erreur
    sprintf(log, "[PID] Erreur Position | X: %.3f | Y: %.3f | Theta: %.3f\r\n",
            erreurPosition.getX(), erreurPosition.getY(), erreurPosition.getTheta());
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Passage repère global -> repère robot
    double errX = erreurPosition.getX();
    double errY = erreurPosition.getY();
    double theta = pointActuel.getTheta();

    double erreurAvant = cos(theta) * errX + sin(theta) * errY;
    double erreurLat   = -sin(theta) * errX + cos(theta) * errY;

    // Terme intégral
    float i1 = this->i[0] + erreurAvant;
    float i2 = this->i[1] + erreurLat;
    float i3 = this->i[2] + this->erreurPosition.getTheta();

    sprintf(log, "[PID] Terme Intégral | iAvant: %.3f | iLat: %.3f | iTheta: %.3f\r\n", i1, i2, i3);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Terme dérivé
    double errX_old = erreurPosition_old.getX();
    double errY_old = erreurPosition_old.getY();

    double erreurAvant_old = cos(theta) * errX_old + sin(theta) * errY_old;
    double erreurLat_old   = -sin(theta) * errX_old + cos(theta) * errY_old;

    double d1 = erreurAvant - erreurAvant_old;
    double d2 = erreurLat - erreurLat_old;
    double d3 = erreurPosition.getTheta() - erreurPosition_old.getTheta();

    sprintf(log, "[PID] Terme Dérivé | dAvant: %.3f | dLat: %.3f | dTheta: %.3f\r\n", d1, d2, d3);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    double erreurTheta = erreurPosition.getTheta();
    constexpr double seuilTheta = 0.05;  // rad ≈ 2.8°
    if (fabs(erreurTheta) < seuilTheta) {
        erreurTheta = 0.0;
        d3 = 0.0;
        i3 = 0.0;
    }

    // Commandes PID (SANS freinage progressif)
    double commandeAvant = kp * erreurAvant + ki * i1 + kd * d1;
    double commandeTheta = Kp_theta * erreurPosition.getTheta() + Ki_theta * i3 + Kd_theta * d3;

    sprintf(log, "[PID] Commandes PID | Avant: %.3f | Theta: %.3f\r\n", commandeAvant, commandeTheta);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Conversion en vitesse
    double vitesseLineaire = commandeAvant;
    double vitesseAngulaire = commandeTheta;

    sprintf(log, "[PID] Vitesses Avant Saturation | Linéaire: %.3f | Angulaire: %.3f\r\n", vitesseLineaire, vitesseAngulaire);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    // Conversion en vitesse des roues
    double vitesseGauche = vitesseLineaire - (this->L / 2) * vitesseAngulaire;
    double vitesseDroite = vitesseLineaire + (this->L / 2) * vitesseAngulaire;

    const double tolPosition = 0.01; // 1 cm
    const double tolTheta = 0.1;    // 0.05 rad ≈ 3°

    if (fabs(erreurAvant) > tolPosition || fabs(erreurLat) > tolPosition || fabs(this->erreurPosition.getTheta()) > tolTheta) {
        this->i[0] = i1;
        this->i[1] = i2;
        this->i[2] = i3;
    }

    // Saturation
    vitesseGauche = std::max(-0.235, std::min(0.235, vitesseGauche));
    vitesseDroite = std::max(-0.235, std::min(0.235, vitesseDroite));

    sprintf(log, "[SET] VITESSE SORTIE DE PID POS | G: %.3f m/s | D: %.3f m/s\r\n", vitesseGauche, vitesseDroite);
    CDC_Transmit_FS((uint8_t*)log, strlen(log));

    if (fabs(erreurAvant) < 0.1 && fabs(erreurLat) < 0.1 && fabs(erreurPosition.getTheta()) < 2) {
        sprintf(log, "[PID] OBJECTIF ATTEINT — Robot à l'arrêt\r\n");
        CDC_Transmit_FS((uint8_t*)log, strlen(log));
        return {0.0, 0.0};
    }

    return {vitesseGauche, vitesseDroite};
}






