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

	robotDrive1->exitSafeStart();
	robotDrive2->exitSafeStart();

	robotDrive1->setMotorSpeed(motLeft);
	robotDrive2->setMotorSpeed(motRight);
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void GlobalPlanner::openRobotsDrive(std::string port1, std::string port2 ){
	//closeRobotsDrive();
	//robotDrive = new RobotDrive(port);
	cout << "Left: " << port1 << ", right: " << port2 << endl;
	robotDrive1 = new Drivers(115200, port1);
	robotDrive2 = new Drivers(115200, port2);
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
