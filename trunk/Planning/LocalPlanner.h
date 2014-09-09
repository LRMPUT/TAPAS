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
//TinyXML
#include <tinyxml.h>
//TAPAS
#include "RobotDrivers/robotDrivers.h"


class Robot;
class Debug;
class GlobalPlanner;

using namespace std;
using namespace cv;

#define HIST_ALPHA 10
#define HIST_SECTORS (360/HIST_ALPHA)
#define HIST_THRESHOLD 0
#define STEERING_MARGIN 4 // in degrees



class LocalPlanner {
	friend class Debug;

public:
	struct Parameters {
		int runThread;
		int avoidObstacles;
		int histResolution;
		float threshold;
		float steeringMargin;
	};

private:
	Parameters localPlannerParams;

	//parent class robot
	Robot* robot;

	//Drivers* robotDrive1;
	//Drivers* robotDrive2;

	GlobalPlanner* globalPlanner;

	bool startOperate;
	bool runThread;


	std::thread localPlannerThread;
	std::mutex driverMtx;

	std::mutex mtxHistSectors;

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

	LocalPlanner(Robot* irobot, GlobalPlanner* planer, TiXmlElement* settings);
	virtual ~LocalPlanner();

	void readSettings(TiXmlElement* settings);

	void stopThread();

	void executeVFH();

	float getLocalDirection();

	void localPlanerTest();

	void startLocalPlanner();
	void stopLocalPlanner();

	std::vector<float> getVecFieldHist();
	//	void setMotorsVel(float motLeft, float motRight);

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
//	void openRobotsDrive(std::string port1, std::string port2);

//	void closeRobotsDrive();

//	bool isRobotsDriveOpen();*/
};

#include "GlobalPlanner.h"
#include "../Robot/Robot.h"

#endif /* LOCALPLANNER_H_ */
