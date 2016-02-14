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
#include "PlannerHelpers.h"


class Robot;
class Debug;
class GlobalPlanner;

//using namespace std;
//using namespace cv;


class LocalPlanner {
	friend class Debug;

private:
	PlannerHelpers::Parameters localPlannerParams;
	int numHistSectors;

	//parent class robot
	Robot* robot;

	//Drivers* robotDrive1;
	//Drivers* robotDrive2;

	GlobalPlanner* globalPlanner;

	bool startOperate;

	std::mutex mtxCurSpeed;
	float curSpeedMax;

	float prevCurSpeed;

	std::thread localPlannerThread;

	std::mutex mtxVecFieldHist;
	std::vector<float> vecFieldHist;
	float shGoalDirLocalMap;
	float shBestDirLocalMap;

	void run();

	void determineDriversCommand(cv::Mat posRobotMapCenter,
								float bestDirLocalMap);

	float determineCurSpeed();

	float setGoalDirection(cv::Mat posLocalToGlobalMap);

	void readSettings(TiXmlElement* settings);

	void executeVFH();

public:

	LocalPlanner(Robot* irobot, GlobalPlanner* planer, TiXmlElement* settings);
	virtual ~LocalPlanner();


	void stopThread();

	//float getLocalDirection();

	void localPlanerTest();

	void startLocalPlanner();
	void stopLocalPlanner();

	void setPreciseSpeed();
	void setNormalSpeed();

	void getVecFieldHist(std::vector<float>& retVecFieldHist, float& retGoalDirection, float& retBestDirection);
};

#include "GlobalPlanner.h"
#include "../Robot/Robot.h"

#endif /* LOCALPLANNER_H_ */
