/*
 * Hokuyo.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <urg_c/urg_sensor.h>
#include "Hokuyo.h"

Hokuyo::Hokuyo() {

}

Hokuyo::~Hokuyo() {

}

void Hokuyo::openPort(std::string port){
	urg_open(&hokuyo, URG_SERIAL, port.c_str(), 115200);
}

void Hokuyo::closeHokuyo(){
	urg_close(&hokuyo);
}

bool Hokuyo::isHokuyoOpen(){
	return hokuyo.is_active;
}

//CV_32SC1 2x1440: x, y points from left to right
cv::MatHokuyo:: getData(){

}
