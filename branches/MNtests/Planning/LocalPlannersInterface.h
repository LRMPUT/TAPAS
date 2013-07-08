/*
 * LocalPlannersInterface.h
 *
 *  If we have multiple local planner I believe it would be good to have a super class
 *  defining what each of them needs to implement and return
 */

#ifndef LOCALPLANNERSINTERFACE_H_
#define LOCALPLANNERSINTERFACE_H_

class LocalPlannersInterface {

public:
	virtual void calculateAction() = 0;
};

#endif /* LOCALPLANNERSINTERFACE_H_ */
