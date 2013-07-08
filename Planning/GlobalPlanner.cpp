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
	delete robotDrive;
	robotDrive = NULL;
}

bool GlobalPlanner::isRobotsDriveOpen(){
	return (robotDrive != NULL);
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
cv::Mat GlobalPlanner::getEncoderData(){
	Mat ret(2, 1, CV_32SC1);
	int enc[2];
	robotDrive->getEncoder(&enc[0], &enc[1]);
	ret.at<int32_t>(0) = enc[LEFT_CHANNEL - 1];
	ret.at<int32_t>(1) = enc[RIGHT_CHANNEL - 1];
	return ret;
}
