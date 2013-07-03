/*
 * GlobalPlanner.h
 *
 * Wanted it to be the global planner containing information about global target and
 * running appropriate local planners
 */

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

#include "../Robot.h"

class GlobalPlanner {
	//parent class robot
	Robot* robot;
public:
	GlobalPlanner(Robot* irobot);
	virtual ~GlobalPlanner();

	//----------------------MENAGMENT OF GlobalPlanner DEVICES
	//Robots Drive
	void openRobotsDrive(std::string port);

	void closeRobotsDrive();

	bool isRobotsDriveOpen();

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32UC1 2x1: left, right encoder
	cv::Mat getEncoderData();
};

#endif /* GLOBALPLANNER_H_ */
