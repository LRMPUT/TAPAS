/*
 * GlobalPlanner.h
 *
 * Wanted it to be the global planner containing information about global target and
 * running appropriate local planners
 */

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

#include <opencv2/opencv.hpp>
//Trobot
#include "../Trobot/include/RobotDrive.h"
#include "RobotDrivers/robotDrivers.h"
//Robots Intellect

class Robot;
class Debug;

enum OperationMode{
	Manual,
	Autonomous
};

class GlobalPlanner {
	friend class Debug;

	//parent class robot
	Robot* robot;

	//Driver
	trobot::RobotDrive* robotDrive;

	Drivers* robotDrive1;
	Drivers* robotDrive2;

	OperationMode currentMode;
public:
	GlobalPlanner(Robot* irobot);
	virtual ~GlobalPlanner();

	//----------------------MODES OF OPERATION
	void switchMode(OperationMode mode);

	void setMotorsVel(float motLeft, float motRight);

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port1, std::string port2);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();
};

#include "../Robot/Robot.h"

#endif /* GLOBALPLANNER_H_ */
