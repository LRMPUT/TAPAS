/*
 * GlobalPlanner.h
 *
 * Wanted it to be the global planner containing information about global target and
 * running appropriate local planners
 */

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

//STL
#include <thread>
#include <mutex>
#include <chrono>
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot
//#include "../Trobot/include/RobotDrive.h"
//TAPAS
#include "RobotDrivers/robotDrivers.h"


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
	//trobot::RobotDrive* robotDrive;

	Drivers* robotDrive1;
	Drivers* robotDrive2;

	OperationMode currentMode;

	bool startOperate;

	bool runThread;
	std::thread globalPlannerThread;

	std::mutex driverMtx;

	void run();

	void processHomologation();
public:
	GlobalPlanner(Robot* irobot);
	virtual ~GlobalPlanner();

	void stopThread();

	void startHomologation();
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
