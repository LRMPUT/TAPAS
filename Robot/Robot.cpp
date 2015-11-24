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

#include <iostream>
#include "Robot.h"

using namespace std;

Robot::Robot(boost::filesystem::path settingsFile) :
		positionEstimation(NULL),
		movementConstraints(NULL),
		globalPlanner(NULL),
		globalPlannerBeingDeleted(false)
{
	cout << "Robot()" << endl;
	TiXmlDocument settings(settingsFile.c_str());
	if(!settings.LoadFile()){
//		cout << "Settings file: " << settingsFile << endl;
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
//	globalPlannerBeingDeleted = true;
	cout << "globalPlanner->stopThread()" << endl;
	globalPlanner->stopThread();
	cout << "movementConstraints->stopThread()" << endl;
	movementConstraints->stopThread();
	cout << "positionEstimation->stopThread()" << endl;
	positionEstimation->stopThread();
	cout << "delete globalPlanner" << endl;
	delete globalPlanner;
	cout << "delete movementConstraints" << endl;
	delete movementConstraints;
	cout << "delete positionEstimation" << endl;
	delete positionEstimation;
	//positionEstimation.stopThread();
	cout << "End ~Robot()" << endl;
}

void Robot::startCompetition() {
	globalPlanner->startCompetition();
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

bool Robot::isGpsDataValid()
{
	return positionEstimation->isGpsDataValid();
}

double Robot::getPosX(double latitude)
{
	return positionEstimation->getPosX(latitude);
}

double Robot::getPosLatitude(double X)
{
	return positionEstimation->getPosLatitude(X);
}

double Robot::getPosY(double longitude)
{
	return positionEstimation->getPosY(longitude);
}

double Robot::getPosLongitude(double Y)
{
	return positionEstimation->getPosLongitude(Y);
}

void Robot::fakeGPSStart(double lat, double lon)
{
	positionEstimation->fakeGPSStart(lat,lon);
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

bool Robot::isImuDataValid(){
	return positionEstimation->isImuDataValid();
}

float Robot::getImuAccVariance() {
	return positionEstimation->getImuAccVariance();
}

//Encoders
void Robot::openEncoders(std::string port){
#ifndef TROBOT
	positionEstimation->openEncoders(port);
#endif
}

void Robot::closeEncoders(){
#ifndef TROBOT
	positionEstimation->closeEncoders();
#endif
}

bool Robot::isEncodersOpen(){
#ifdef TROBOT
	if(globalPlanner != NULL){
		return globalPlanner->isEncodersOpen();
	}
	else{
		return false;
	}
#else
	return positionEstimation->isEncodersOpen();
#endif
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
cv::Mat Robot::getEncoderData(std::chrono::high_resolution_clock::time_point& timestamp){
#ifdef TROBOT
	if(globalPlannerBeingDeleted){
		cout << "empty Mat" << endl;
		return Mat(2, 1, CV_32SC1, Scalar(0));
	}
	else{
//		cout << "globalPlanner->getEncoderData" << endl;
		return globalPlanner->getEncoderData(timestamp);
	}
#else
	return positionEstimation->getEncoderData(timestamp);
#endif
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

void Robot::getLocalPlanData(cv::Mat& constraintsMap,cv::Mat& posRobotMapCenter, cv::Mat& globalMapCenter){
	movementConstraints->getLocalPlanningData(constraintsMap, posRobotMapCenter, globalMapCenter);
}
