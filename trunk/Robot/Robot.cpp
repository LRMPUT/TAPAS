/*
 * Robot.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */
#include <iostream>
#include "Robot.h"

using namespace std;

Robot::Robot(boost::filesystem::path settingsFile) {
	cout << "Robot()" << endl;
	TiXmlDocument settings(settingsFile.c_str());
	if(!settings.LoadFile()){
		throw settings.ErrorDesc();
	}
	TiXmlElement* pRobot;
	pRobot = settings.FirstChildElement("Robot");
	if(!pRobot){
		throw "Bad settings file - entry Robot not found";
	}

	//Must be first, because MovementConstraints uses it's devices
	positionEstimation = new PositionEstimation(this, pRobot);

	TiXmlElement* pMovementConstraints = pRobot->FirstChildElement("MovementConstraints");
	movementConstraints = new MovementConstraints(this, pMovementConstraints);

	globalPlanner = new GlobalPlanner(this, pRobot);

	startTime = std::chrono::high_resolution_clock::now();
	cout << "End Robot()" << endl;
}

Robot::~Robot() {
	cout << "~Robot()" << endl;
	cout << "delete globalPlanner" << endl;
	delete globalPlanner;
	cout << "delete positionEstimation" << endl;
	delete positionEstimation;
	cout << "delete movementConstraints" << endl;
	delete movementConstraints;
	//positionEstimation.stopThread();
	cout << "End ~Robot()" << endl;
}


void Robot::homologation(){
	globalPlanner->startHomologation();
}

//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void Robot::openRobotsDrive(std::string port1, std::string port2){
	globalPlanner->openRobotsDrive(port1, port2);
}

void Robot::closeRobotsDrive(){
	globalPlanner->closeRobotsDrive();
}

bool Robot::isRobotsDriveOpen(){
	return globalPlanner->isRobotsDriveOpen();
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void Robot::openGps(std::string port){
	positionEstimation->openGps(port);
}

void Robot::closeGps(){
	positionEstimation->closeGps();
}

bool Robot::isGpsOpen(){
	return positionEstimation->isGpsOpen();
}

bool Robot::isSetZero() {
	positionEstimation->isSetZero();
}

int Robot::gpsGetFixStatus()
{
	return positionEstimation->gpsGetFixStatus();
}

double Robot::getPosX(double longitude)
{
	return positionEstimation->getPosX(longitude);
}

double Robot::getPosLongitude(double X)
{
	return positionEstimation->getPosLongitude(X);
}

double Robot::getPosY(double latitude)
{
	return positionEstimation->getPosY(latitude);
}

double Robot::getPosLatitude(double Y)
{
	return positionEstimation->getPosLatitude(Y);
}

//Imu
void Robot::openImu(std::string port){
	positionEstimation->openImu(port);
}


void Robot::closeImu(){
	positionEstimation->closeImu();
}

bool Robot::isImuOpen(){
	return positionEstimation->isImuOpen();
}

//Encoders
void Robot::openEncoders(std::string port){
	positionEstimation->openEncoders(port);
}

void Robot::closeEncoders(){
	positionEstimation->closeEncoders();
}

bool Robot::isEncodersOpen(){
	return positionEstimation->isEncodersOpen();
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
//CV_32SC1 2x1: left, right encoder
cv::Mat Robot::getEncoderData(){
	std::chrono::high_resolution_clock::time_point timestamp;
	return positionEstimation->getEncoderData(timestamp);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
cv::Mat Robot::getImuData(std::chrono::high_resolution_clock::time_point &timestamp){
	return positionEstimation->getImuData(timestamp);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat Robot::getEstimatedPosition(){
	return positionEstimation->getEstimatedPosition();
}

//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
cv::Mat Robot::getMovementConstraints(){
	return movementConstraints->getMovementConstraints();
}


cv::Mat Robot::getPosImuConstraintsMapCenter(){
	return movementConstraints->getPosImuMapCenter();
}

//cv::Mat Robot::getLocalMapPosInGlobalMap(){
//	return movementConstraints->getPosGlobalMap();
//}

void Robot::getLocalPlanData(cv::Mat& MovementConstraints,cv::Mat& PosImuMapCenter, cv::Mat& GlobalMapCenter){
	movementConstraints->getLocalPlanningData(MovementConstraints, PosImuMapCenter, GlobalMapCenter);
}
