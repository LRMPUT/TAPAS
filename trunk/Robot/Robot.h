/*
 * Robot.h
 *
 * Wanted it to be main class containing simple: update data, plan and do actions
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "../PositionEstimation/PositionEstimation.h"
#include "../Planning/GlobalPlanner.h"

class Robot {

	// Class containing information about our position estimation from sensors
	PositionEstimation positionEstimate;

	// Class responsible for planning
	GlobalPlanner globalPlanner;


public:
	Robot();
	virtual ~Robot();


};

#endif /* ROBOT_H_ */
