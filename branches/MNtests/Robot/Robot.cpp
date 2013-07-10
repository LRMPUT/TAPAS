/*
 * Robot.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include "Robot.h"

Robot::Robot() : globalPlanner(this), movementConstraints(this), positionEstimation(this) {


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
	movementConstraints.openHokuyo(port);
}

void Robot::closeHokuyo(){
	movementConstraints.closeHokuyo();
}

bool Robot::isHokuyoOpen(){
	return movementConstraints.isHokuyoOpen();
}

//Camera
void Robot::openCamera(){
	movementConstraints.openCamera();
}

void Robot::closeCamera(){
	movementConstraints.closeCamera();
}

bool Robot::isCameraOpen(){
	return movementConstraints.isCameraOpen();
}


//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32UC1 2x1: left, right encoder
cv::Mat Robot::getEncoderData(){
	return globalPlanner.getEncoderData();
}

//CV_32SC1 2x1: x, y position
cv::Mat Robot::getGpsData(){
	return positionEstimation.getGpsData();
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
cv::Mat Robot::getImuData(){
	return positionEstimation.getImuData();
}

//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
cv::Mat Robot::getHokuyoData(){
	return movementConstraints.getHokuyoData();
}

//CV_8UC3 2x640x480: left, right image
cv::Mat Robot::getCameraData(){
	return movementConstraints.getCameraData();
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
cv::Mat Robot::getEstimatedPosition(){
	return positionEstimation.getEstimatedPosition();
}

//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
cv::Mat Robot::getMovementConstraints(){
	return movementConstraints.getMovementConstraints();
}
