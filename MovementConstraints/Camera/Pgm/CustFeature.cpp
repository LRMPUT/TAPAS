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
#include "../Pgm/CustFeature.h"

#include <iostream>
#include <cmath>


using namespace std;

//-------------ISING PAIRWISE-------------

IsingPairFeature::IsingPairFeature(int iid, int iparamNum/*, int iobsVecBeg, int iobsVecEnd*/) :
	Feature(iid, iparamNum/*, iobsVecBeg, iobsVecEnd*/)
{

}

double IsingPairFeature::comp(const std::vector<double>& vals,
							const std::vector<double>& obsVec)
{
	return vals[0]*vals[1];
}

double IsingPairFeature::compParam(const std::vector<double>& vals,
								const std::vector<double>& params,
								const std::vector<double>& obsVec)
{
	/*cout << "Feature, paramNum() = " << paramNum() << endl;
	cout << "vals[0] = " << vals[0] << endl;
	cout << "vals[1] = " << vals[1] << endl;
	cout << "params[paramNum()] = " << params[paramNum()] << endl;*/
	return params[paramNum()]*vals[0]*vals[1];
}

//-------------ISING NODE-------------

IsingNodeFeature::IsingNodeFeature(int iid, int iparamNum/*, int iobsVecBeg, int iobsVecEnd*/) :
	Feature(iid, iparamNum/*, iobsVecBeg, iobsVecEnd*/)
{

}

double IsingNodeFeature::comp(const std::vector<double>& vals,
							const std::vector<double>& obsVec)
{
	if(obsVec.empty()){
		return vals[0];
	}
	return vals[0]*obsVec[0];
}

double IsingNodeFeature::compParam(const std::vector<double>& vals,
								const std::vector<double>& params,
								const std::vector<double>& obsVec)
{
	/*cout << "Feature, paramNum() = " << paramNum() << endl;
	cout << "vals[0] = " << vals[0] << endl;
	cout << "params[paramNum()] = " << params[paramNum()] << endl;*/
	if(obsVec.empty()){
		return params[paramNum()]*vals[0];
	}
	return params[paramNum()]*vals[0]*obsVec[0];
}

//-------------TERRAIN CLASSIFICATION NODE-------------

TerClassNodeFeature::TerClassNodeFeature(int iid, int iparamNum, const std::vector<int>& iobsNums) :
		Feature(iid, iparamNum, iobsNums)
{

}

double TerClassNodeFeature::comp(const std::vector<double>& vals,
								const std::vector<double>& obsVec)
{
	int label = (int)(vals[0] + 0.5);
	return log(obsVec[obsNums()[label]])/4;
}

double TerClassNodeFeature::compParam(const std::vector<double>& vals,
						const std::vector<double>& params,
						const std::vector<double>& obsVec)
{
	return params[paramNum()]*comp(vals, obsVec);
}

//-------------TERRAIN CLASSIFICATION PAIRWISE-------------

TerClassPairFeature::TerClassPairFeature(int iid, int iparamNum, const std::vector<int>& iobsNums) :
		Feature(iid, iparamNum, iobsNums)
{

}

double TerClassPairFeature::comp(const std::vector<double>& vals,
								const std::vector<double>& obsVec)
{
	static const double beta = 0.05;
	int ind = (fabs(vals[0] - vals[1]) < 1e-4 ? 0 : 1);
	double diff = obsVec[obsNums()[0]] - obsVec[obsNums()[1]];
	return ind * exp(-beta * diff * diff);
}

double TerClassPairFeature::compParam(const std::vector<double>& vals,
						const std::vector<double>& params,
						const std::vector<double>& obsVec)
{
	return params[paramNum()] * comp(vals, obsVec);
}



