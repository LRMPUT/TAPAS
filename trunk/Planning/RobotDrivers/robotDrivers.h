/*
 * robotDrivers.h
 *
 *  Created on: Mar 27, 2014
 *      Author: sebastian
 */

#ifndef ROBOTDRIVERS_H_
#define ROBOTDRIVERS_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "../../Trobot/include/SerialPort.h"

class Drivers {

	trobot::SerialPort* serialPort;

	boost::circular_buffer<char> driverResp;

	//bool commandCorrect;

public:
	Drivers();
	Drivers(unsigned int baud, const std::string device);

	virtual ~Drivers();

	void exitSafeStart();

	void setMotorSpeed(int speed);

	void stopMotor();


};


#endif /* ROBOTDRIVERS_H_ */
