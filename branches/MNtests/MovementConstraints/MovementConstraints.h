/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

#include <string>
#include <opencv2/opencv.hpp>
//#include "Camera/***"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"
#include "../Robot/Robot.h"


class MovementConstaints {

	// Class to get data from Camera

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Sharp
	Sharp sharp;

	//Parent class Robot
	Robot* robot;

public:
	//MovementConstaints(Robot* irobot);
	MovementConstaints();
	virtual ~MovementConstaints();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1440: x, y points from left to right
	cv::Mat getHokuyoData();

	//CV_8UC3 2x640x480: left, right image
	cv::Mat getCamerasData();

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCameras();

	void closeCameras();

	bool areCamerasOpen();
};

#endif /* MOVEMENTCONSTAINTS_H_ */
