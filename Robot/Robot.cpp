/*
 * Robot.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */
#include <iostream>
#include "Robot.h"

using namespace std;

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

	startTime = std::chrono::high_resolution_clock::now();
}

Robot::~Robot() {
	positionEstimation.stopThread();
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void Robot::openRobotsDrive(std::string port1, std::string port2){
	globalPlanner.openRobotsDrive(port1, port2);
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

//Encoders
void Robot::openEncoders(std::string port){
	positionEstimation.openEncoders(port);
}

void Robot::closeEncoders(){
	positionEstimation.closeEncoders();
}

bool Robot::isEncodersOpen(){
	return positionEstimation.isEncodersOpen();
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
void Robot::openCamera(std::vector<std::string> device){
	movementConstraints->openCamera(device);
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
	std::chrono::high_resolution_clock::time_point timestamp;
	return positionEstimation.getEncoderData(timestamp);
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
