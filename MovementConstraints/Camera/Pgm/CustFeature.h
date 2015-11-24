/*
	Copyright (c) 2015,	TAPAS Team:
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
