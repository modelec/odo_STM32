#ifndef INC_PID_POSITION_H_
#define INC_PID_POSITION_H_

#include "pid.h"

class PidPosition : public Pid {
private:
    Point erreurPosition, erreurPosition_old;
    Point consignePositionFinale;

    float d[3] = {0.0f, 0.0f, 0.0f};			//valeurs calculé dans les méthodes update (position)
    float i[3] = {0.0f, 0.0f, 0.0f};

    float Kp_theta, Ki_theta, Kd_theta;

public:
    // Constructeur
    PidPosition(float kp, float ki, float kd,
                float Kp_theta, float Ki_theta, float Kd_theta,
                Point consignePosition);

    // Setters
    void setConsignePositionFinale(Point consigne);
    void setErreurPosition(Point erreur);
    void setErreurPosition_old(Point erreur_old);

    void setD(float d[2]);
    void setI(float d[2]);

    void setKpTheta(float kp);
    void setKiTheta(float ki);
    void setKdTheta(float kd);

    // Getters
    Point getConsignePositionFinale();
    Point getErreurPosition();
    Point getErreurPosition_old();

    float getKpTheta();
    float getKiTheta();
    float getKdTheta();

    std::array<float, 2> getD();
    std::array<float, 2> getI();

    // Méthodes spécifiques à la position
    Point calculErreurPosition(Point p);
    void updateErreurPosition(Point pointActuel);
    std::array<double, 2> updateNouvelOrdreVitesse(Point pointActuel, float vitGauche, float vitDroit);

    // Surcharge des méthodes virtuelles
    void updateErreur();
    void updateNouvelleConsigne();
};

#endif /* INC_PID_POSITION_H_ */
