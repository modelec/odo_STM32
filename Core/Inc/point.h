/*
 * point.h
 *
 *  Created on: Mar 24, 2025
 *      Author: CHAUVEAU Maxime
 */

#ifndef INC_POINT_H_
#define INC_POINT_H_

#include <array>
#include <cstdint>

typedef enum StatePoint {
    INTERMEDIAIRE,
    FINAL,
    NONDETERMINE
}StatePoint;

class Point {
private:
    float x;
    float y;
    float theta;
    uint32_t id;
    StatePoint state;

public:
    // Constructeur
    Point(float x = 0.0, float y = 0.0, float theta = 0.0, StatePoint state = StatePoint::INTERMEDIAIRE);

    // Setters
    void setX(float x);
    void setY(float y);
    void setTheta(float theta);
    void setState(StatePoint s);
    void setID(uint32_t id);

    // Getters
    float getX();
    float getY();
    float getTheta();
    std::array<float, 3> getPoint();
    StatePoint getState();
    uint32_t getID();
};



#endif /* INC_POINT_H_ */
