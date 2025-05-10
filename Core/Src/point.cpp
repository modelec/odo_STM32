

#include "point.h"

Point::Point(float x, float y, float theta, StatePoint state){
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->state = state;
}

void Point::setX(float x){
	this->x = x;
}

void Point::setY(float y){
	this->y = y;
}

void Point::setTheta(float theta){
	this->theta = theta;
}

void Point::setState(StatePoint s){
	this->state = s;
}

void Point::setID(uint32_t id){
	this->id = id;
}


float Point::getX(){
	return x;
}

float Point::getY(){
	return y;
}

float Point::getTheta(){
	return theta;
}

std::array<float, 3> Point::getPoint(){
	return {this->x, this->y, this->theta};
}

StatePoint Point::getState(){
	return state;
}

uint32_t Point::getID(){
	return id;
}
