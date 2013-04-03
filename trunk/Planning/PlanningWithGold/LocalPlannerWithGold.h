/*
 * LocalPlannerWithGold.h
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#ifndef LOCALPLANNERWITHGOLD_H_
#define LOCALPLANNERWITHGOLD_H_

#include "../LocalPlannersInterface.h"

class LocalPlannerWithGold: public LocalPlannersInterface {
public:
	LocalPlannerWithGold();
	virtual ~LocalPlannerWithGold();

	void calculateAction();
};

#endif /* LOCALPLANNERWITHGOLD_H_ */
