/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

//STL
#include <string>
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//RobotsIntellect
#include "Camera/Camera.h"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"
#include "Kinect/Kinect.h"

class Robot;
class Debug;

class MovementConstraints {
	friend class Debug;

	// Class to get data from Camera
	Camera* camera;

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Kinect
	Kinect kinect;

	// Class to get data from Sharp
	Sharp sharp;

	//Parent class Robot
	Robot* robot;

public:
	MovementConstraints(Robot* irobot, TiXmlElement* settings);
	virtual ~MovementConstraints();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	const cv::Mat getHokuyoData();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera(std::vector<std::string> device);

	void closeCamera();

	bool isCameraOpen();
};

#include "../Robot/Robot.h"

#endif /* MOVEMENTCONSTAINTS_H_ */
