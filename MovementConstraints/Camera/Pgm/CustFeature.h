/*
 * CustFeature.h
 *
 *  Created on: 6 sty 2015
 *      Author: jachu
 */

#ifndef CUSTFEATURE_H_
#define CUSTFEATURE_H_

#include "../Pgm/Pgm.h"

//------------------ISING------------------

class IsingPairFeature : public Feature
{

public:
	IsingPairFeature(int iid, int iparamNum/*, int iobsVecBeg = -1, int iobsVecEnd = -1*/);

	virtual double comp(const std::vector<double>& vals,
			const std::vector<double>& obsVec = std::vector<double>());

	virtual double compParam(const std::vector<double>& vals,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());
};

class IsingNodeFeature : public Feature
{

public:
	IsingNodeFeature(int iid, int iparamNum/*, int iobsVecBeg = -1, int iobsVecEnd = -1*/);

	virtual double comp(const std::vector<double>& vals,
			const std::vector<double>& obsVec = std::vector<double>());

	virtual double compParam(const std::vector<double>& vals,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());
};

//------------------Terrain classification------------------

class TerClassNodeFeature : public Feature
{

public:
	TerClassNodeFeature(int iid, int iparamNum, const std::vector<int>& iobsNums = std::vector<int>());

	virtual double comp(const std::vector<double>& vals,
			const std::vector<double>& obsVec = std::vector<double>());

	virtual double compParam(const std::vector<double>& vals,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());
};

class TerClassPairFeature : public Feature
{

public:
	TerClassPairFeature(int iid, int iparamNum, const std::vector<int>& iobsNums = std::vector<int>());

	virtual double comp(const std::vector<double>& vals,
			const std::vector<double>& obsVec = std::vector<double>());

	virtual double compParam(const std::vector<double>& vals,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());
};


#endif /* CUSTFEATURE_H_ */
