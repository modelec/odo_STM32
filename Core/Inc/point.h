/*
 * point.h
 *
 *  Created on: Mar 24, 2025
 *      Author: Modelec
 */

#ifndef INC_POINT_H_
#define INC_POINT_H_

#include <cstdint>

enum StatePoint {
    INTERMEDIAIRE = 0,
    FINAL = 1,
    NONDETERMINE = 2
};

class Point {
public:
    uint8_t id;
    StatePoint state;

    float x;
    float y;
    float theta;
    bool active = false;

    Point(uint8_t id = 0, StatePoint state = StatePoint::INTERMEDIAIRE, float x = 0.0, float y = 0.0, float theta = 0.0, bool active = false);
};

#endif /* INC_POINT_H_ */
