/*
 * CustFeature.cpp
 *
 *  Created on: 6 sty 2015
 *      Author: jachu
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
	return log(obsVec[label])/3;
}

double TerClassNodeFeature::compParam(const std::vector<double>& vals,
						const std::vector<double>& params,
						const std::vector<double>& obsVec)
{
	int label = (int)(vals[0] + 0.5);
//	cout << "obsVec[0] = " << obsVec[0] << endl;
//	cout << "obsVec[1] = " << obsVec[1] << endl;
	return params[paramNum()]*log(obsVec[label])/3;
}

//-------------TERRAIN CLASSIFICATION PAIRWISE-------------

TerClassPairFeature::TerClassPairFeature(int iid, int iparamNum, const std::vector<int>& iobsNums) :
		Feature(iid, iparamNum, iobsNums)
{

}

double TerClassPairFeature::comp(const std::vector<double>& vals,
								const std::vector<double>& obsVec)
{
	static const double beta = 2;
	int ind = (fabs(vals[0] - vals[1]) < 1e-4 ? 0 : 1);
	double diff = obsVec[obsNums()[0]] - obsVec[obsNums()[1]];
	return ind * exp(-beta * diff * diff);
}

double TerClassPairFeature::compParam(const std::vector<double>& vals,
						const std::vector<double>& params,
						const std::vector<double>& obsVec)
{
	static const double beta = 2;
	int ind = (fabs(vals[0] - vals[1]) < 1e-4 ? 0 : 1);
	double diff = obsVec[obsNums()[0]] - obsVec[obsNums()[1]];
	return params[paramNum()] * ind * exp(-beta * diff * diff);
}



