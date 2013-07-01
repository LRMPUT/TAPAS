/*
 * Robot.h
 *
 * Wanted it to be main class containing simple: update data, plan and do actions
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "../PositionEstimation/PositionEstimation.h"
#include "../Planning/GlobalPlanner.h"
#include <fstream>
#include <string>

class Robot {

	// Class containing information about our position estimation from sensors
	PositionEstimation positionEstimate;

	// Class responsible for planning
	GlobalPlanner globalPlanner;


public:
	Robot();
	virtual ~Robot();

	void switchToManualMode();

	void switchToAutonomousMode();

	void setMotorsVel(float mot1, float mot2);

	//Robots Drive
	void openRobotsDrive(std::string port);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();

	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	//Hokuyo
	void openHokuyo();

	void closeHokuyo();

	bool isHokuyoOpen();

	//Camera
	void openCamera();

	void closeCamera();

	bool isCameraOpen();

	//Imu
	void openImu(std::string port);

	void closeImu();

	bool isImuClosed();

	//left, right encoder
	void getEncoderData(std::vector<int>& data);

	//x, y position
	void getGpsData(std::vector<int>& data);

	//acc, gyro, magnet, euler
	void getImuData(std::vector<float>& data);

	//x, y points
	void getHokuyoData(std::vector<int>& data);

	//left, right image
	void getCameraData(std::vector<cv::Mat>& data);

};

#endif /* ROBOT_H_ */
