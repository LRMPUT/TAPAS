/*
 * GlobalPlanner.h
 *
 * Wanted it to be the global planner containing information about global target and
 * running appropriate local planners
 */

#ifndef GLOBALPLANNER_H_
#define GLOBALPLANNER_H_

class GlobalPlanner {
public:
	GlobalPlanner();
	virtual ~GlobalPlanner();
};

#endif /* GLOBALPLANNER_H_ */
