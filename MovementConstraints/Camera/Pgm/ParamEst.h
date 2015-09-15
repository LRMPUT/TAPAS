/*
 * ParamEst.h
 *
 *  Created on: 30 gru 2014
 *      Author: jachu
 */

#ifndef PARAMEST_H_
#define PARAMEST_H_

#include <vector>
#include <thread>

#include "../Pgm/Pgm.h"

class ParamEst{
	std::vector<Pgm>* pgm;
	const std::vector<std::vector<double> >* varVals;
	const std::vector<std::vector<double> >* obsVec;
	const std::vector<int>* paramMap;
	std::vector<double> params;

public:
	ParamEst();

	void evaluate(const std::vector<double>& paramValsMapped,
					double& likelihood,
					std::vector<double>& grad);

	void estimateParams(std::vector<Pgm>& curPgm,
						const std::vector<std::vector<double> >& curVarVals,
						const std::vector<std::vector<double> >& curObsVec = std::vector<std::vector<double> >(),
						const std::vector<int>& curParamMap = std::vector<int>());
};

class EvaluateThread{
	Pgm& curPgm;
	const std::vector<double>& curObsVec;
	const std::vector<double>& curVarVals;
	const std::vector<double>& paramVals;
	double& logPartFunc;
	double& likelihoodNum;
	std::vector<double>& Efi;
	std::vector<double>& Ed;
	std::vector<std::vector<std::vector<double> > >& msgs;
	bool separateMarg;

	std::thread runThread;
	bool finished;

	void run();

public:
	EvaluateThread(Pgm& icurPgm,
					const std::vector<double>& icurObsVec,
					const std::vector<double>& icurVarVals,
					const std::vector<double>& iparamVals,
					double& ilogPartFunc,
					double& ilikelihoodNum,
					std::vector<double>& iEfi,
					std::vector<double>& iEd,
					std::vector<std::vector<std::vector<double> > >& imsgs,
					bool iseparateMarg);

	~EvaluateThread();

	bool hasFinished();

};

#endif /* PARAMEST_H_ */
