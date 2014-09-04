/*
 * LocalPlanner.h
 *
 */

#ifndef LOCALPLANNER_H_
#define LOCALPLANNER_H_

#include <iostream>
//STL
#include <thread>
#include <mutex>
#include <chrono>
//OpenCV
#include <opencv2/opencv.hpp>
//TAPAS
#include "RobotDrivers/robotDrivers.h"
#include "GlobalPlanner.h"

class Robot;
class Debug;

using namespace std;
using namespace cv;

#define HIST_ALPHA 10
#define HIST_SECTORS (360/HIST_ALPHA)
#define HIST_THRESHOLD 0
#define STEERING_MARGIN 4 // in degrees



class LocalPlanner {
	friend class Debug;

	//parent class robot
	Robot* robot;

	//Drivers* robotDrive1;
	//Drivers* robotDrive2;

	GlobalPlanner* globalPlanner;

	bool startOperate;
	bool runThread;


	std::thread localPlannerThread;
	std::mutex driverMtx;

	void run();

	vector<float> histSectors;
	vector<int> freeSectors;
	float bestDirection;
	float goalDirection;

	//constraint data
	Mat constraints;
	Mat posImuMapCenter;
	Mat posLocalToGlobalMap;

	void initHistogram();

	void updateHistogram();

	void smoothHistogram();

	void findFreeSectors();

	void determineDriversCommand();

	void findOptimSector();

	void determineGoalInLocalMap();

	float RotMatToEulerYaw(Mat rotMat);

	void setGoalDirection();

public:
	LocalPlanner(Robot* irobot, GlobalPlanner* planer);
	virtual ~LocalPlanner();

	void stopThread();

	void executeVFH();

	float getLocalDirection();

	void localPlanerTest();

	//	void setMotorsVel(float motLeft, float motRight);

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
//	void openRobotsDrive(std::string port1, std::string port2);

//	void closeRobotsDrive();

//	bool isRobotsDriveOpen();*/
};

#include "../Robot/Robot.h"

#endif /* LOCALPLANNER_H_ */
