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
	cout << "~GlobalPlanner()" << endl;
	stopThread();
	closeRobotsDrive();
	cout << "End ~GlobalPlanner()" << endl;
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
	std::chrono::seconds duration(10);
	std::this_thread::sleep_for(duration);
	cout << "Pause end" << endl;

	bool runForward = true;
	static const float distance = 10000;
	static const int motorsVel = 1000;
	float prevDist = robot->getPosImuConstraintsMapCenter().at<float>(0, 3);
	float distRun = 0;
	while(runForward && runThread){
		Mat constraints = robot->getMovementConstraints();
		Mat posImuMapCenter = robot->getPosImuConstraintsMapCenter();

		bool wayBlocked = false;

		float addX = posImuMapCenter.at<float>(0, 3)/MAP_RASTER_SIZE;
		for(int x = MAP_SIZE/2; x < MAP_SIZE/2 + 10; x++){
			for(int y = MAP_SIZE/2 - 3; y < MAP_SIZE/2 + 3; y++){
				if(x + addX >= 0 && x + addX < MAP_SIZE){
					if(constraints.at<float>(x + addX, y) > 0.5){
						wayBlocked = true;
						break;
					}
				}
			}
		}

		cout << "wayBlocked = " << wayBlocked << endl;
		if(wayBlocked == false){
			cout << "setting " << motorsVel << endl;
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

void GlobalPlanner::stopThread(){
	runThread = false;
	if(globalPlannerThread.joinable()){
		globalPlannerThread.join();
	}
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

	std::unique_lock<std::mutex> lck(driverMtx);
	if(robotDrive1 != NULL && robotDrive2 != NULL){
		robotDrive1->exitSafeStart();
		robotDrive2->exitSafeStart();

		robotDrive1->setMotorSpeed(motLeft);
		robotDrive2->setMotorSpeed(-motRight);
	}
	lck.unlock();
}
//----------------------ACCESS TO COMPUTED DATA

GlobalPlanner::GlobalPlanInfo GlobalPlanner::getGlobalPlan(){
	GlobalPlanInfo info;
	return info;
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void GlobalPlanner::openRobotsDrive(std::string port1, std::string port2 ){
	//closeRobotsDrive();
	//robotDrive = new RobotDrive(port);
	cout << "Left: " << port1 << ", right: " << port2 << endl;

	closeRobotsDrive();
	std::unique_lock<std::mutex> lck(driverMtx);
	robotDrive1 = new Drivers(115200, port1);
	robotDrive2 = new Drivers(115200, port2);
	lck.unlock();

	setMotorsVel(0, 0);
}

void GlobalPlanner::closeRobotsDrive(){
	cout << "closeRobotsDrive()" << endl;
	std::unique_lock<std::mutex> lck(driverMtx);
	if(robotDrive1 != NULL){
		robotDrive1->exitSafeStart();
		robotDrive1->setMotorSpeed(0);
		delete robotDrive1;
		robotDrive1 = NULL;
	}
	if(robotDrive2 != NULL){
		robotDrive2->exitSafeStart();
		robotDrive2->setMotorSpeed(0);
		delete robotDrive2;
		robotDrive2 = NULL;
	}
	lck.unlock();
	cout << "End closeRobotsDrive()" << endl;
}

bool GlobalPlanner::isRobotsDriveOpen(){
	return (robotDrive1 != NULL) && (robotDrive2 != NULL) ;
}
