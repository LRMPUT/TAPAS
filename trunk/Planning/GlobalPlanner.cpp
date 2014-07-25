/*
 * GlobalPlanner.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

//STL
#include <iostream>
#include <chrono>
//OpenCV
#include <opencv2/opencv.hpp>
//#include "../Trobot/include/RobotDrive.h"
//TAPAS
#include "GlobalPlanner.h"
//#include "../MovementConstraints/MovementConstraints.h"

using namespace cv;
using namespace std;

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

GlobalPlanner::GlobalPlanner(Robot* irobot) :
		robot(irobot),
		robotDrive1(NULL),
		robotDrive2(NULL),
		startOperate(false)
{
	runThread = true;
	globalPlannerThread = std::thread(&GlobalPlanner::run, this);
}

GlobalPlanner::~GlobalPlanner(){
	closeRobotsDrive();
	runThread = false;
	globalPlannerThread.join();
}

void GlobalPlanner::run(){
	while(runThread){
		if(startOperate){
			processHomologation();
		}
		std::chrono::milliseconds duration(50);
		std::this_thread::sleep_for(duration);
	}
}

void GlobalPlanner::processHomologation(){
	cout << "homologation()" << endl;
	if(!isRobotsDriveOpen()){
		cout << "Robots drive not open, ending homologation" << endl;
		return;
	}
	cout << "Pause" << endl;
	//60 seconds pause
	std::chrono::seconds duration(60);
	//std::this_thread::sleep_for(duration);
	cout << "Pause end" << endl;

	bool runForward = true;
	static const float distance = 10000;
	static const int motorsVel = 1000;
	float prevDist = robot->getPosImuConstraintsMapCenter().at<float>(0, 3);
	float distRun = 0;
	while(runForward){
		Mat constraints = robot->getMovementConstraints();
		Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

		bool wayBlocked = false;

		for(int x = MAP_SIZE/2; x < MAP_SIZE/2 + 10; x++){
			for(int y = MAP_SIZE/2 - 3; y < MAP_SIZE/2 + 3; y++){
				if(constraints.at<float>(x, y) > 0.5){
					wayBlocked = true;
					break;
				}
			}
		}

		if(wayBlocked == false){
			setMotorsVel(motorsVel, motorsVel);
		}
		else{
			setMotorsVel(0, 0);
		}

		float curDist = posImuMapCenter.at<float>(0, 3);
		if(curDist < prevDist){
			distRun += curDist + (MAP_SIZE/2 - MAP_MARGIN)*MAP_RASTER_SIZE - prevDist;
		}
		else{
			distRun += curDist - prevDist;
		}
		prevDist = curDist;
		cout << "distRun = " << distRun << endl;
		if(distRun > distance){
			setMotorsVel(0, 0);
			runForward = false;
		}

		std::chrono::milliseconds duration(200);
		std::this_thread::sleep_for(duration);
	}
	cout << "Position = " << robot->getPosImuConstraintsMapCenter() << endl;
	startOperate = false;
}

void GlobalPlanner::startHomologation(){
	startOperate = true;
}

//----------------------MODES OF OPERATION
void GlobalPlanner::switchMode(OperationMode mode){
	currentMode = mode;
	setMotorsVel(0 , 0);
}

void GlobalPlanner::setMotorsVel(float motLeft, float motRight){

	robotDrive1->exitSafeStart();
	robotDrive2->exitSafeStart();

	robotDrive1->setMotorSpeed(motLeft);
	robotDrive2->setMotorSpeed(-motRight);
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void GlobalPlanner::openRobotsDrive(std::string port1, std::string port2 ){
	//closeRobotsDrive();
	//robotDrive = new RobotDrive(port);
	cout << "Left: " << port1 << ", right: " << port2 << endl;
	robotDrive1 = new Drivers(115200, port1);
	robotDrive2 = new Drivers(115200, port2);
	setMotorsVel(0, 0);
}

void GlobalPlanner::closeRobotsDrive(){
	if(robotDrive1 != NULL){
		delete robotDrive1;
		robotDrive1 = NULL;
	}
	if(robotDrive2 != NULL){
		delete robotDrive2;
		robotDrive2 = NULL;
	}
}

bool GlobalPlanner::isRobotsDriveOpen(){
	return (robotDrive1 != NULL) && (robotDrive2 != NULL) ;
}
