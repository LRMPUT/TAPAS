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

	OperationMode currentMode;
public:
	GlobalPlanner(Robot* irobot);
	virtual ~GlobalPlanner();

	//----------------------MODES OF OPERATION
	void switchMode(OperationMode mode);

	void setMotorsVel(float motLeft, float motRight);

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();
};

#include "../Robot/Robot.h"

#endif /* GLOBALPLANNER_H_ */
