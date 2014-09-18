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

//#define HIST_ALPHA 10
//#define HIST_SECTORS (360/HIST_ALPHA)
//#define HIST_THRESHOLD 0
//#define STEERING_MARGIN 4 // in degrees



class LocalPlanner {
	friend class Debug;

public:
	struct Parameters {
		int runThread;
		int debug;
		int avoidObstacles;
		int histResolution;
		float threshold;
		float steeringMargin;
		float gauss3sig;
		float maxDistance;
		float backwardsPenalty;
		float normalSpeed;
		float preciseSpeed;
		float gentleTurnMargin;
		float gentleTurnDiff;
		float turnSpeed;
		int turnTimeout;
		int interruptTime;
		float interruptSpeed;
		float imuAccVarianceLimit;
	};

private:
	Parameters localPlannerParams;
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

	void updateHistogram(std::vector<float>& histSectors,
						cv::Mat posRobotMapCenter,
						cv::Mat constraints);

	void smoothHistogram(std::vector<float>& histSectors);

//	void findFreeSectors(std::vector<float>& histSectors,
//						std::vector<int>& freeSectors);

	float determineGoalInLocalMap(cv::Mat posLocalToGlobalMap,
								float goalDirGlobalMap);

	float findOptimSector(const std::vector<float>& histSectors,
							cv::Mat posRobotMapCenter,
							float goalDirLocalMap);

	void determineDriversCommand(cv::Mat posRobotMapCenter,
								float bestDirLocalMap);

	float determineCurSpeed();

	float rotMatToEulerYaw(Mat rotMat);

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
