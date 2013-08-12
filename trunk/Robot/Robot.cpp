/*
 * Robot.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */
#include <iostream>
#include "Robot.h"

Robot::Robot(boost::filesystem::path settingsFile) : globalPlanner(this), positionEstimation(this) {
	TiXmlDocument settings(settingsFile.c_str());
	if(!settings.LoadFile()){
		throw settings.ErrorDesc();
	}
	TiXmlElement* pRobot;
	pRobot = settings.FirstChildElement("Robot");
	if(!pRobot){
		throw "Bad settings file - entry Robot not found";
	}
	TiXmlElement* pMovementConstraints = pRobot->FirstChildElement("MovementConstraints");
	movementConstraints = new MovementConstraints(this, pMovementConstraints);
}

Robot::~Robot() {

}

//----------------------MODES OF OPERATION
void Robot::switchMode(OperationMode mode){
	globalPlanner.switchMode(mode);
}

void Robot::setMotorsVel(float motLeft, float motRight){
	globalPlanner.setMotorsVel(motLeft, motRight);
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void Robot::openRobotsDrive(std::string port){
	globalPlanner.openRobotsDrive(port);
}

void Robot::closeRobotsDrive(){
	globalPlanner.closeRobotsDrive();
}

bool Robot::isRobotsDriveOpen(){
	return globalPlanner.isRobotsDriveOpen();
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void Robot::openGps(std::string port){
	positionEstimation.openGps(port);
}

void Robot::closeGps(){
	positionEstimation.closeGps();
}

bool Robot::isGpsOpen(){
	return positionEstimation.isGpsOpen();
}

//Imu
void Robot::openImu(std::string port){
	positionEstimation.openImu(port);
}

void Robot::closeImu(){
	positionEstimation.closeImu();
}

bool Robot::isImuOpen(){
	return positionEstimation.isImuOpen();
}

//----------------------MENAGMENT OF MovementConstraints DEVICES
//Hokuyo
void Robot::openHokuyo(std::string port){
	movementConstraints->openHokuyo(port);
}

void Robot::closeHokuyo(){
	movementConstraints->closeHokuyo();
}

bool Robot::isHokuyoOpen(){
	return movementConstraints->isHokuyoOpen();
}

//Camera
void Robot::openCamera(){
	movementConstraints->openCamera();
}

void Robot::closeCamera(){
	movementConstraints->closeCamera();
}

bool Robot::isCameraOpen(){
	return movementConstraints->isCameraOpen();
}


//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32UC1 2x1: left, right encoder
const cv::Mat Robot::getEncoderData(){
	return globalPlanner.getEncoderData();
}

//CV_32SC1 2x1: x, y position
const cv::Mat Robot::getGpsData(){
	return positionEstimation.getGpsData();
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
const cv::Mat Robot::getImuData(){
	return positionEstimation.getImuData();
}

//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
const cv::Mat Robot::getHokuyoData(){
	return movementConstraints->getHokuyoData();
}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Robot::getCameraData(){
	return movementConstraints->getCameraData();
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat Robot::getEstimatedPosition(){
	return positionEstimation.getEstimatedPosition();
}

//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
const cv::Mat Robot::getMovementConstraints(){
	return movementConstraints->getMovementConstraints();
}
