/*
 * Robot.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include "Robot.h"

Robot::Robot() {
	// TODO Auto-generated constructor stub

}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}


//----------------------MENAGMENT OF GlobalPlanner DEVICES
//Robots Drive
void Robot::openRobotsDrive(std::string port)
{

}

void Robot::closeRobotsDrive()
{

}

bool Robot::isRobotsDriveOpen()
{
	return true;
}

//----------------------MENAGMENT OF PositionEstimation DEVICES
//Gps
void Robot::openGps(std::string port)
{

}

void Robot::closeGps()
{

}

bool Robot::isGpsOpen()
{
	return true;
}

//Imu
void Robot::openImu(std::string port)
{

}

void Robot::closeImu()
{

}

bool Robot::isImuOpen()
{
	return true;
}

