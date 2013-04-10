/*
 * LocalPlannerSearching.h
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#ifndef LOCALPLANNERSEARCHING_H_
#define LOCALPLANNERSEARCHING_H_

#include "../LocalPlannersInterface.h"

class LocalPlannerSearching : public LocalPlannersInterface{
public:
	LocalPlannerSearching();
	virtual ~LocalPlannerSearching();

	void calculateAction();
};

#endif /* LOCALPLANNERSEARCHING_H_ */
