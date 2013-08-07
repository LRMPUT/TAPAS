/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Camera/Camera.h"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"

class Robot;

class MovementConstraints {

	// Class to get data from Camera
	Camera* camera;

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Sharp
	Sharp sharp;

	//Parent class Robot
	Robot* robot;

public:
	MovementConstraints(Robot* irobot);
	virtual ~MovementConstraints();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
	const cv::Mat getHokuyoData();

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getCameraData();

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera();

	void closeCamera();

	bool isCameraOpen();
};

#include "../Robot/Robot.h"

#endif /* MOVEMENTCONSTAINTS_H_ */
