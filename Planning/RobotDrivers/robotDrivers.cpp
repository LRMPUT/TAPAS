/*
 * robotDrivers.cpp
 *
 *  Created on: Mar 27, 2014
 *      Author: sebastian
 */

#include "robotDrivers.h"
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cv;
using namespace trobot;



Drivers::Drivers() {


}

Drivers::Drivers(unsigned int baud, const std::string device) {

	serialPort = new SerialPort(baud, device);

}

Drivers::~Drivers() {

}

void Drivers::exitSafeStart(){

	serialPort->write("GO\r");

}

void Drivers::setMotorSpeed(float speed){
	if (speed > 0)
	serialPort->write('F' + boost::lexical_cast<string>(speed) + '\r');
	else
	serialPort->write('R' + boost::lexical_cast<string>(-speed) + '\r');
}

void Drivers::stopMotor(){

	serialPort->write("X\r");
}

