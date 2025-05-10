#ifndef INC_PIDVITESSE_H_
#define INC_PIDVITESSE_H_

#include "pid.h"

class PidVitesse : public Pid {
private:
    float erreurVitesse, erreurVitesse_old;
    float consigneVitesseFinale;
    float derivee, integral;
    float nouvelleConsigneVitesse;

public:
    // Constructeur
    PidVitesse(float kp, float ki, float kd, float consigneVitesseFinale);

    // Setters
    void setErreurVitesse(float e);
    void setErreurVitesse_old(float e_old);
    void setConsigneVitesseFinale(float vf);
    void setNouvelleConsigneVitesse(float v);
    void setDerivee(float d);
    void setIntegral(float i);

    // Getters
    float getErreurVitesse();
    float getErreurVitesse_old();
    float getConsigneVitesseFinale();
    float getNouvelleConsigneVitesse();
    float getDerivee();
    float getIntegral();

    // Méthodes spécifiques au PID vitesse
    float calculErreurVitesse(float vitesseActuelle);
    void updateErreurVitesse(float vitesseActuelle);
    void updateNouvelleVitesse(float vitesseActuelle);

};

#endif /* INC_PIDVITESSE_H_ */
