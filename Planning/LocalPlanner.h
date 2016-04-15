
/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "PlannerHelpers.h"
//ROs
#include "ros/ros.h"
#include "TAPAS/PlanningData.h"

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

	cv::Mat constraints;
	cv::Mat posRobotMapCenter;
	cv::Mat posLocalToGlobalMap;

	ros::NodeHandle nh;

	ros::Subscriber planning_sub;

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

	void planningCallback(const TAPAS::PlanningData& msg);

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
