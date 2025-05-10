/*
 * pid.h
 *
 *  Created on: Mar 25, 2025
 *      Author: CHAUVEAU Maxime
 */

#ifndef INC_PID_H_
#define INC_PID_H_
#include "point.h"
#include <cmath>
#include <array>

/*
 Pour notre robot, on va faire 3 PID. On va avoir un premier PID qui prend une position et qui
 doit donner en sortie deux ordres pour les moteurs (un moteur gauche et un moteur droit).
 Ainsi on aura aussi un PID par moteur qui eux recevront la consigne du pid position, donc 3 en tout.

 Le premier PID (position) prend en entrée un Point de la carte (objectif) et renvoie en sortie un tableau de 2 val en m/s.
 les pid vitesse de chaque moteurs prennent en entrée une vitesse à atteindre et en sortie une consigne à envoyer au moteur (qu'il faudra traduire en pwm).
 */

class Pid {
protected:

	float Vmax;			//vitesse linéaire max du robot (mps)
	float Wmax;			//vitesse angulaire max du robot (mps)
	float L;			//Largeur entre les deux roues (en mètre)

    float kp;
    float ki;
    float kd;




public:
    // Constructeur
    Pid(float kp, float ki, float kd);

    // Setters
    void setKp(float k);
    void setKi(float k);
    void setKd(float k);

    // Getters
    float getKp();
    float getKi();
    float getKd();

    //Methodes
    int getPWMCommand(float vitesse);			//convertir des m/s en notre signal pwm
};



#endif /* INC_PID_H_ */
