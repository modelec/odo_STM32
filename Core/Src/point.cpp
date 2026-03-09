

#include "point.h"

Point::Point(uint8_t id, StatePoint state, float x, float y, float theta, bool active, bool isAtPosition) : id(id), state(state), x(x), y(y), theta(theta), active(active), isAtPosition(isAtPosition) {
}
