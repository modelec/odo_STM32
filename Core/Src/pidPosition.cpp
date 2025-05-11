
#include "pidPosition.h"


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
Point PidPosition::calculErreurPosition(Point p) {
    return Point(this->consignePositionFinale.getX() - p.getX(),
                 this->consignePositionFinale.getY() - p.getY(),
                 this->consignePositionFinale.getTheta() - p.getTheta(),
                 StatePoint::NONDETERMINE);
}

// Mise à jour et calcul du nouvel ordre de vitesse pour les moteurs
std::array<double, 2> PidPosition::updateNouvelOrdreVitesse(Point pointActuel) {
	/*
		 La sortie est calculée en combinant les 3 termes :
		 	 - proportionnel : on multiplie kp par l'erreur
		 	 - integral 	 : on multiplie ki par la somme des erreurs passées
		 	 - derivee		 : on multiplie kd par la variation de l'erreur
	*/
    this->updateErreurPosition(pointActuel);

    // Mise à jour du terme intégral
    this->i[0] += this->erreurPosition.getX();
    this->i[1] += this->erreurPosition.getY();
    this->i[2] += this->erreurPosition.getTheta();

    // Calcul du terme dérivé
    this->d[0] = this->erreurPosition.getX() - this->erreurPosition_old.getX();
    this->d[1] = this->erreurPosition.getY() - this->erreurPosition_old.getY();
    this->d[2] = this->erreurPosition.getTheta() - this->erreurPosition_old.getTheta();

    // Calcul du PID pour chaque axe
    double commandeX = kp * this->erreurPosition.getX() + ki * this->i[0] + kd * this->d[0];
    double commandeY = kp * this->erreurPosition.getY() + ki * this->i[1] + kd * this->d[1];
    double commandeTheta = Kp_theta * this->erreurPosition.getTheta() + Ki_theta * this->i[2] + Kd_theta * this->d[2];

    // Conversion des commandes X/Y en une vitesse linéaire et une rotation
    double vitesseLineaire = sqrt(commandeX * commandeX + commandeY * commandeY);
    double vitesseAngulaire = commandeTheta;

    // Écrêtage des valeurs
    if (vitesseLineaire > this->Vmax){
    	vitesseLineaire = this->Vmax;
    }
    if (vitesseLineaire < -this->Vmax){
       	vitesseLineaire = -this->Vmax;
    }
    if (vitesseAngulaire > this->Wmax){
    	vitesseAngulaire = this->Wmax;
    }
    if (vitesseAngulaire < -this->Wmax){
    	vitesseAngulaire = -this->Wmax;
    }

    // Conversion en vitesse des roues
    double vitesseGauche = vitesseLineaire - (this->L / 2) * vitesseAngulaire;
    double vitesseDroite = vitesseLineaire + (this->L / 2) * vitesseAngulaire;

    //return {vitesseGauche, vitesseDroite};
    return {0.5, 0.5};
}
