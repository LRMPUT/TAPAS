/*
 * Robot.h
 *
 * Wanted it to be main class containing simple: update data, plan and do actions
 */

#ifndef ROBOT_H_
#define ROBOT_H_

//STL
#include <fstream>
#include <string>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>

class Robot;
class Debug;

//RobotsIntellect
#include "../PositionEstimation/PositionEstimation.h"
#include "../MovementConstraints/MovementConstraints.h"
#include "../Planning/GlobalPlanner.h"

#define RASTER_SIZE 200		//[mm]
#define MAP_SIZE	10000/RASTER_SIZE	//[u] 10m

class Robot {
	friend class Debug;

	// Class containing information about our position estimation from sensors
	PositionEstimation positionEstimation;

	// Class responsible for planning
	GlobalPlanner globalPlanner;

	MovementConstraints* movementConstraints;

public:
	Robot(boost::filesystem::path settings);
	virtual ~Robot();

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port1, std::string port2);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();

	//----------------------MENAGMENT OF PositionEstimation DEVICES
	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	//Imu
	void openImu(std::string port);

	void closeImu();

	bool isImuOpen();

	//Encoders
	void openEncoders(std::string port);

	void closeEncoders();

	bool isEncodersOpen();

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera(std::vector<std::string> device);

	void closeCamera();

	bool isCameraOpen();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1: left, right encoder
	const cv::Mat getEncoderData();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	const cv::Mat getEstimatedPosition();

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();
};

#endif /* ROBOT_H_ */
