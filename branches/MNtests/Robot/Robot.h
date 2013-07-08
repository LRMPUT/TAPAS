/*
 * Robot.h
 *
 * Wanted it to be main class containing simple: update data, plan and do actions
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

#include "../PositionEstimation/PositionEstimation.h"
#include "../Planning/GlobalPlanner.h"


#define RASTER_SIZE 200		//[mm]
#define MAP_SIZE	10000/RASTER_SIZE	//[u] 10m

class Robot {

	// Class containing information about our position estimation from sensors
	PositionEstimation positionEstimate;

	// Class responsible for planning
	GlobalPlanner globalPlanner;


public:
	Robot();
	virtual ~Robot();

	//----------------------MODES OF OPERATION
	void switchToManualMode();

	void switchToAutonomousMode();

	void setMotorsVel(float mot1, float mot2);

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port);

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

	//----------------------MENAGMENT OF MovementConstraints DEVICES
	//Hokuyo
	void openHokuyo(std::string port);

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCameras();

	void closeCameras();

	bool areCamerasOpen();


	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32UC1 2x1: left, right encoder
	cv::Mat getEncoderData();

	//CV_32SC1 2x1: x, y position
	cv::Mat getGpsData();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
	cv::Mat getImuData();

	//CV_32SC1 2x1440: x, y points from left to right
	cv::Mat getHokuyoData();

	//CV_8UC3 2x640x480: left, right image
	cv::Mat getCamerasData();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	cv::Mat getEstimatedPosition();

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robots position (MAP_SIZE/2, 0)
	cv::Mat getMovementConstraints();
};

#endif /* ROBOT_H_ */