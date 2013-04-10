/*
 * LocalPlannerToGold.h
 *
 * It needs to implement LocalPlannersIntefrace methods !!!
 */

#ifndef LOCALPLANNERTOGOLD_H_
#define LOCALPLANNERTOGOLD_H_

#include "../LocalPlannersInterface.h"

class LocalPlannerToGold : public LocalPlannersInterface{
public:
	LocalPlannerToGold();
	virtual ~LocalPlannerToGold();

	void calculateAction();
};

#endif /* LOCALPLANNERTOGOLD_H_ */
