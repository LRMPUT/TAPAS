/*
 * GlobalPlanner.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <opencv2/opencv.hpp>
#include "../Trobot/include/RobotDrive.h"
#include "GlobalPlanner.h"

using namespace trobot;
using namespace cv;
using namespace std;

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

GlobalPlanner::GlobalPlanner(Robot* irobot) : robot(irobot), robotDrive(NULL) {

}

GlobalPlanner::~GlobalPlanner(){
	closeRobotsDrive();
}

//----------------------MODES OF OPERATION
void GlobalPlanner::switchMode(OperationMode mode){
	currentMode = mode;
	setMotorsVel(0 , 0);
}

void GlobalPlanner::setMotorsVel(float motLeft, float motRight){
	robotDrive->runMotor(motLeft, LEFT_CHANNEL);
	robotDrive->runMotor(motRight, RIGHT_CHANNEL);
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void GlobalPlanner::openRobotsDrive(std::string port){
	closeRobotsDrive();
	robotDrive = new RobotDrive(port);
}

void GlobalPlanner::closeRobotsDrive(){
	if(robotDrive != NULL){
		delete robotDrive;
		robotDrive = NULL;
	}
}

bool GlobalPlanner::isRobotsDriveOpen(){
	return (robotDrive != NULL);
}
