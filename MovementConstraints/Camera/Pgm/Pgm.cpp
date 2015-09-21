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

#include "../Pgm/Pgm.h"

#include <iostream>
#include <cmath>



using namespace std;

std::vector<double> featScales;

//----------------RandVar----------------

RandVar::RandVar(int iid, std::vector<double> ivals) :
		idData(iid),
		isObserved(false),
		valsData(ivals)
{

}

RandVar::RandVar(int iid, double observedVal) :
		idData(iid),
		isObserved(true)
{
	valsData = vector<double>(1, observedVal);
}

void RandVar::makeObserved(double val)
{
	isObserved = true;
	valsData = vector<double>{val};
}

void RandVar::makeNotObserved(std::vector<double> vals)
{
	isObserved = false;
	valsData = vals;
}

void RandVar::setVals(const std::vector<double>& newVals)
{
	valsData = newVals;
}

//----------------Feature----------------

Feature::Feature(int iid, int iparamNum, const std::vector<int>& iobsNums) :
	idData(iid),
	paramNumData(iparamNum),
	obsNumsData(iobsNums)
//	obsVecBeg(iobsVecBeg),
//	obsVecEnd(iobsVecEnd)
{

}

Feature::~Feature(){

}

//----------------Cluster----------------

void Cluster::normalizeMarg(std::vector<double>& marg,
							MargType type) const
{
	//normalization
	if(type == SumProduct){
		double sumVal = 0;
		for(int v = 0; v < (int)marg.size(); ++v){
			sumVal += marg[v];
		}
		for(int v = 0; v < (int)marg.size(); ++v){
			marg[v] /= sumVal;
		}
	}
	else if(type == MaxSum){
		double maxVal = -1e9;
		for(int v = 0; v < (int)marg.size(); ++v){
			maxVal = max(marg[v], maxVal);
		}

		double sumVal = 0;
		for(int v = 0; v < (int)marg.size(); ++v){
			sumVal += exp(marg[v] - maxVal);
		}

		double normVal = log(1.0) - (maxVal + log(sumVal));

//		cout << "sumVal = " << sumVal << ", log(1.0/sumVal) = " << log(1.0/sumVal) << endl;
		for(int v = 0; v < (int)marg.size(); ++v){
			marg[v] += normVal;
		}
	}
}

Cluster::Cluster(int iid,
			const std::vector<Feature*>& ifeats,
			const std::vector<RandVar*>& irandVars /*id sorted*/,
			const std::vector<int>& irandVarsOrder,
			const std::vector<int>& iobsVecIdxs) :
	idData(iid),
	featuresData(ifeats),
	randVarsData(irandVars),
	randVarsOrderData(irandVarsOrder),
	obsVecIdxsData(iobsVecIdxs)
{

}

void Cluster::setNh(std::vector<Cluster*> newNh /*id sorted*/){
//	cout << "nhData.size() = " << nhData.size() << endl;
//	cout << "newNh.size() = " << newNh.size() << endl;
////	nhData = vector<Cluster*>();
//	vector<Cluster*> a = newNh;
//	cout << "setting" << endl;
	nhData = newNh;
//	cout << "end setting" << endl;
}

void Cluster::setSepsets(std::vector<std::vector<RandVar*> > newSepsets /*id sorted*/){
	sepsetsData = newSepsets;
}

double Cluster::compFactorsVal(const std::vector<double>& varVals,
							MargType type,
							const std::vector<double>& params,
							const std::vector<double>& obsVec) const
{
	double ret = 0;
	vector<double> curObsVec = getCurObsVec(obsVec);
	vector<double> varValsOrdered(varVals.size());
	for(int rv = 0; rv < int(randVarsOrderData.size()); ++rv){
		varValsOrdered[randVarsOrderData[rv]] = varVals[rv];
	}
	for(int f = 0; f < (int)featuresData.size(); ++f){
		double val = featuresData[f]->compParam(varValsOrdered, params, curObsVec);
		ret += val;
	}
	if(type == SumProduct){
		return exp(ret);
	}
	else if(type == MaxSum){
		return ret;
	}

	return exp(ret);
}

double Cluster::compSumHcdiv(const std::vector<std::vector<double> >& inMsgs,
									const std::vector<double>& params,
									const std::vector<double>& obsVec)
{
	vector<double> marg = marginalize(vector<RandVar*>(),
										SumProduct,
										inMsgs,
										params,
										obsVec);
	int numVars = randVarsData.size();
	int numFeats = featuresData.size();
	double Hcdiv = 0.0;
	{
		vector<int> varValIdxs(numVars, 0);
		int numVarValIdxs = 0;

		//compute curObsVec
		vector<double> curObsVec = getCurObsVec(obsVec);

		double sumMsg = 0.0;
		do{
	//				cout << "numVarValIdxs = " << numVarValIdxs << endl;
			vector<double> varValsOrdered(numVars, 0);
			for(int v = 0; v < numVars; ++v){
				varValsOrdered[randVarsOrderData[v]] = randVarsData[v]->vals()[varValIdxs[v]];
			}
			double curMarg = marg[numVarValIdxs];

			double factorVal = 0;
			for(int f = 0; f < (int)featuresData.size(); ++f){
				factorVal += featuresData[f]->compParam(varValsOrdered, params, curObsVec);
			}
			factorVal = exp(factorVal);
//			cout << "curMarg = " << curMarg << ", factorVal = " << factorVal << endl;

			sumMsg += curMarg;

			Hcdiv += curMarg * log(curMarg/factorVal);

			numVarValIdxs += 1;
		}while(Pgm::incVarValIdxs(varValIdxs, randVarsData));

//		cout << "sumMsg = " << sumMsg << endl;
	}

//	double HcdivComp = 0.0;
//	double normConstComp = 0.0;
//
//	{
//		vector<int> varValsIdxs(randVarsData.size(), 0);
//
//		do{
//			double curMarg = 1.0;
//			vector<double> varVals(randVarsData.size());
//			for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
//				varVals[rv] = randVarsData[rv]->vals()[varValsIdxs[rv]];
//			}
//
//			for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
//				curMarg *= inMsgs[rv][varValsIdxs[rv]];
//			}
//
//			double factVal = compFactorsVal(varVals, SumProduct, params, obsVec);
//			curMarg *= factVal;
//
//			normConstComp += curMarg;
//		}while(Pgm::incVarValIdxs(varValsIdxs, randVarsData));
//
//		normConstComp = 1.0/normConstComp;
//
//		cout << "normConstComp = " << normConstComp << endl;
//	}
//
//	{
//		double sumMsg = 0.0;
//
//		vector<int> varValsIdxs(randVarsData.size(), 0);
//
//		do{
//			double curMarg = 1.0;
//			vector<double> varVals(randVarsData.size());
//			for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
//				varVals[rv] = randVarsData[rv]->vals()[varValsIdxs[rv]];
//			}
//
//			for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
//				curMarg *= inMsgs[rv][varValsIdxs[rv]];
//			}
//
//			double factVal = compFactorsVal(varVals, SumProduct, params, obsVec);
//			curMarg *= factVal;
//
//			sumMsg += curMarg*normConstComp;
//
//			HcdivComp += curMarg*normConstComp*log(curMarg*normConstComp/factVal);
//		}while(Pgm::incVarValIdxs(varValsIdxs, randVarsData));
//
//		cout << "sumMsg = " << sumMsg << endl;
//		cout << "HcdivComp = " << HcdivComp << endl;
//	}
//
//	cout << "Hcdiv = " << Hcdiv << endl;
//	char a;
//	cin >> a;

	return Hcdiv;
}

std::vector<double> Cluster::compSumModelExpectation(const std::vector<std::vector<double> >& inMsgs,
															const std::vector<double>& params,
															const std::vector<double>& obsVec)
{
	vector<double> marg = marginalize(vector<RandVar*>(),
										SumProduct,
										inMsgs,
										params,
										obsVec);
	int numVars = randVarsData.size();
	int numFeats = featuresData.size();

	vector<double> Efi(numFeats, 0.0);

	vector<int> varValIdxs(randVarsData.size(), 0);
	int numVarValIdxs = 0;
	vector<double> curObsVec = getCurObsVec(obsVec);

	do{
//					cout << "numVarValIdxs = " << numVarValIdxs << endl;
		vector<double> clustVarValsOrdered(randVarsData.size(), 0);

		for(int v = 0; v < (int)randVarsData.size(); ++v){
			clustVarValsOrdered[randVarsOrderData[v]] = randVarsData[v]->vals()[varValIdxs[v]];
		}
		for(int f = 0; f < (int)featuresData.size(); ++f){
			double curVal = featuresData[f]->comp(clustVarValsOrdered,
												curObsVec);

			Efi[f] += curVal*marg[numVarValIdxs];
//						cout << "paramNum = " << paramNum << ", curVal*marg[c][numVarValIdxs] = " <<
//								curVal << "*" << marg[c][numVarValIdxs] << endl;
		}

		numVarValIdxs += 1;
	}while(Pgm::incVarValIdxs(varValIdxs, randVarsData));

	return Efi;
}

std::vector<double> Cluster::compSumEmpiricalExpectation(const std::vector<double>& varVals,
													const std::vector<double>& obsVec) const
{
	int numFeats = featuresData.size();
	vector<double> Ed(numFeats, 0.0);

	vector<double> curObsVec = getCurObsVec(obsVec);
	vector<double> varValsOrdered(varVals.size());
	for(int rv = 0; rv < int(randVarsOrderData.size()); ++rv){
		varValsOrdered[randVarsOrderData[rv]] = varVals[rv];
	}
//	cout << "varValsOrdered = " << varValsOrdered << endl;
//	cout << "curObsVec = " << curObsVec << endl;
	for(int f = 0; f < numFeats; ++f){
		double curVal = featuresData[f]->comp(varValsOrdered, curObsVec);
		Ed[f] = curVal;
	}

	return Ed;
}

 std::vector<std::vector<int> > Cluster::getBestValsIdxs(const std::vector<std::vector<double> >& inMsgs,
																const std::vector<double>& params,
																const std::vector<double>& obsVec,
																double eps)
{
	vector<double> maxVals = marginalize(vector<RandVar*>(),
										MaxSum,
										inMsgs,
										params,
										obsVec);

//	cout << "inMsgs = " << inMsgs << endl;
//	cout << "maxVals = " << maxVals << endl;

	double bestScore = -1e9;
	vector<vector<int> > bestValsIdxs;

	int numVars = randVarsData.size();
	vector<int> varValIdxs(numVars, 0);
	int numVarValIdxs = 0;

	do{
		if(bestScore < maxVals[numVarValIdxs]){
			bestScore = maxVals[numVarValIdxs];
		}

		numVarValIdxs += 1;
	}while(Pgm::incVarValIdxs(varValIdxs, randVarsData));

	varValIdxs = vector<int>(numVars, 0);
	numVarValIdxs = 0;

	do{
		//possible tie
		if(fabs(bestScore - maxVals[numVarValIdxs]) < eps){
//			vector<double> vals(numVars, 0);
//			for(int v = 0; v < numVars; ++v){
//				vals[v] = randVarsData[v]->vals()[varValIdxs[v]];
//			}
			bestValsIdxs.push_back(varValIdxs);
		}

		numVarValIdxs += 1;
	}while(Pgm::incVarValIdxs(varValIdxs, randVarsData));


//		cout << "Max vals for cluster " << c << endl;
//		for(int val = 0; val < maxVals.back().size(); ++val){
//			cout << maxVals.back()[val] << " ";
//		}
//		cout << endl;

//	char a;
//	cin >> a;
	if(bestValsIdxs.size() > 1){
		int maxComb = 1;
		for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
			maxComb *= randVarsData[rv]->vals().size();
		}
		if(maxComb == (int)bestValsIdxs.size()){
			bestValsIdxs = vector<vector<int> >(randVarsData.size(), vector<int>{-1});
		}
	}

	return bestValsIdxs;
}

std::vector<double> Cluster::marginalize(const std::vector<RandVar*>& margVars /*id sorted*/,
									MargType type,
									const std::vector<std::vector<double> >& inMsgs,
									const std::vector<double>& params,
									const std::vector<double>& obsVec,
									const Cluster* excluded)
{
	vector<int> margVarValIdxs(margVars.size(), 0);
	vector<bool> varIsMarg(randVarsData.size(), false);
	vector<int> varPosList(randVarsData.size(), 0);

	vector<double> curObsVec;
	if(!obsVec.empty()){
		for(int o = 0; o < (int)obsVecIdxsData.size(); ++o){
			curObsVec.push_back(obsVec[obsVecIdxsData[o]]);
		}
	}

	//cout << "Creating otherVars and varPosList, randVarsData.size() = " << randVarsData.size() << endl;
	vector<RandVar*> otherVars;
	int posMargVars = 0;
	for(int rv = 0; rv < (int)randVarsData.size(); ++rv){
		if(margVars.size() > 0){
			while(randVarsData[rv]->id() > margVars[posMargVars]->id()){
				if(posMargVars >= (int)margVars.size() - 1){
					break;
				}
				else{
					++posMargVars;
				}
			}
			//cout << "margVars[posMargVars]->id() = " << margVars[posMargVars]->id() <<
			//		", randVarsData[rv]->id() = " << randVarsData[rv]->id() << endl;
			if(margVars[posMargVars]->id() != randVarsData[rv]->id()){
				otherVars.push_back(randVarsData[rv]);
				varPosList[rv] = otherVars.size() - 1;
			}
			else{
				varIsMarg[rv] = true;
				varPosList[rv] = posMargVars;
			}
		}
		else{
			otherVars.push_back(randVarsData[rv]);
			varPosList[rv] = otherVars.size() - 1;
		}
	}
	vector<int> otherVarValIdxs(otherVars.size(), 0);

	//cout << "otherVars.size() = " << otherVars.size() << endl;
	int margLen = 1;
	for(int ov = 0; ov < (int)otherVars.size(); ++ov){
		//cout << "otherVars[ov]->vals().size() = " << otherVars[ov]->vals().size() << endl;
		margLen *= otherVars[ov]->vals().size();
	}

	vector<double> marg;

	if(type == SumProduct){
		marg = vector<double>(margLen, 0.0);
	}
	else if(type == MaxSum){
		marg = vector<double>(margLen, -1e9);
	}
//	cout << "Starting main marginalization loop" << endl;
	int margIdx = 0;
	do{

		do{

			double margCurVal = 1.0;
			if(type == SumProduct){
				margCurVal = 1.0;
			}
			else if(type == MaxSum){
				margCurVal = 0.0;
			}

			vector<double> curVarValsOrdered(randVarsData.size(), 0.0);

//			cout << "randVarsOrderData = " << randVarsOrderData << endl;
			int posCurVarVals = 0;
			int mv = 0; //marginal variable index
			int ov = 0;	//other variable index
			while((mv < (int)margVars.size()) || (ov < (int)otherVars.size())){
//				cout << "posCurVarVals = " << posCurVarVals << endl;
				if((mv < (int)margVars.size()) && (ov < (int)otherVars.size())){
					if(margVars[mv]->id() < otherVars[ov]->id()){
						curVarValsOrdered[randVarsOrderData[posCurVarVals]] = margVars[mv]->vals()[margVarValIdxs[mv]];
						++mv;
					}
					else{
						curVarValsOrdered[randVarsOrderData[posCurVarVals]] = otherVars[ov]->vals()[otherVarValIdxs[ov]];
						++ov;
					}
				}
				else if(mv < (int)margVars.size()){
					curVarValsOrdered[randVarsOrderData[posCurVarVals]] = margVars[mv]->vals()[margVarValIdxs[mv]];
					++mv;
				}
				else if(ov < (int)otherVars.size()){
					curVarValsOrdered[randVarsOrderData[posCurVarVals]] = otherVars[ov]->vals()[otherVarValIdxs[ov]];
					++ov;
				}

				++posCurVarVals;
			}

			//features
//			cout << "Computing features" << endl;
			for(int f = 0; f < (int)featuresData.size(); ++f){

//				cout << "Computing feature id = " << featuresData[f]->id() <<
//						", curVarVals.size() = " << curVarValsOrdered.size() <<
//						", params.size() = " << params.size() <<
//						", curObsVec = " << curObsVec <<
//						", value = " << featuresData[f]->compParam(curVarValsOrdered, params, curObsVec) << endl;
				double exponent = featuresData[f]->compParam(curVarValsOrdered, params, curObsVec);
//				cout << "end computing" << endl;
//				if(id() == 100256){
//					cout << "curVarValsOrdered = " << curVarValsOrdered << endl;
//					cout << "params = " << params << endl;
//					cout << "curObsVec = " << curObsVec << endl;
//					cout << "exponent = " << exponent << endl;
//				}
				if(type == SumProduct){
					margCurVal *= exp(exponent);
				}
				else if(type == MaxSum){
					margCurVal += exponent;
				}
			}
//			cout << "end computing" << endl;

			//received messages
			//TODO possible speed up by precaching
			for(int nhc = 0; nhc < (int)nhData.size(); ++nhc){

				//cout << "Preparing message index from " << nhData[nhc]->id() <<
				//		", nhData.size() = " << nhData.size() << endl;

				if(excluded != 0){
					if(nhData[nhc]->id() == excluded->id()){
						//cout << "Excluding " << nhData[nhc]->id() << endl;
						continue;
					}
				}

				//computing index of message value

				int msgIdx = 0;
				int msgIdxMul = 1;
				int rvPos = 0;
				for(int sep = 0; sep < (int)sepsetsData[nhc].size(); ++sep){
					while(randVarsData[rvPos]->id() < sepsetsData[nhc][sep]->id()){
						if(rvPos >= (int)randVarsData.size() - 1){
							break;
						}
						else{
							++rvPos;
						}
					}

					if(randVarsData[rvPos]->id() == sepsetsData[nhc][sep]->id()){
						int curSepsetVarIdx = 0;
						if(varIsMarg[rvPos]){
							curSepsetVarIdx = margVarValIdxs[varPosList[rvPos]];
						}
						else{
							curSepsetVarIdx = otherVarValIdxs[varPosList[rvPos]];
						}
						msgIdx += msgIdxMul * curSepsetVarIdx;
						msgIdxMul *= sepsetsData[nhc][sep]->vals().size();
					}
				}

//				cout << "Multiplying in message value from cluster " << nhData[nhc]->id() <<
//						", value = " << inMsgs[nhc][msgIdx] << endl;

				if(type == SumProduct){
					margCurVal *= inMsgs[nhc][msgIdx];
				}
				else if(type == MaxSum){
					margCurVal += inMsgs[nhc][msgIdx];
				}
			}

//			if(id() == 29778){
//				cout << "marg[" << margIdx << "] = max(" << margCurVal << ", " << marg[margIdx] << ")" << endl;
//			}

			if(type == SumProduct){
				marg[margIdx] += margCurVal;
			}
			else if(type == MaxSum){
				marg[margIdx] = max(margCurVal, marg[margIdx]);
			}

			//iterate through all combinations of other variables
		} while(Pgm::incVarValIdxs(margVarValIdxs, margVars));

		//iterate through all combinations of marginalized variables
		++margIdx;
//		cout << "margIdx = " << margIdx << endl;
	} while(Pgm::incVarValIdxs(otherVarValIdxs, otherVars));

//	cout << "Before normalization" << endl;
//	for(int v = 0; v < marg.size(); ++v){
//		cout << marg[v] << ", ";
//	}
//	cout << endl;

//	if(id() == 80){
//		cout << "before norm, marg = " << marg << endl;
//	}

	//normalization
	normalizeMarg(marg, type);

//	cout << "After normalization" << endl;
//	for(int v = 0; v < marg.size(); ++v){
//		cout << marg[v] << ", ";
//	}
//	cout << endl;

//	if(id() == 80){
//		cout << "after norm, marg = " << marg << endl;
//
//		cout << "inMsgs = " << inMsgs << endl;
////		char a;
////		cin >> a;
//	}

//	for(int m = 0; m < (int)marg.size(); ++m){
//		if(std::isnan(marg[m]) || std::isinf(marg[m])){
//			cout << "marg " << id() << " = " << marg << endl;
////			cout << "inMsgs = " << inMsgs << endl;
//			char a;
//			cin >> a;
//		}
//	}

	return marg;
}

//----------------Pgm----------------

Pgm::Pgm(std::vector<RandVar*> irandVars,
		std::vector<Cluster*> iclusters,
		std::vector<Feature*> ifeats) :
	randVarsData(irandVars),
	clustersData(iclusters),
	featsData(ifeats)
{

}

bool Pgm::incVarValIdxs(std::vector<int>& varValIdxs,
					const std::vector<RandVar*>& vars)
{
	//iterate through all combinations of variables
	int pos = 0;
	bool carry = true;
	while(pos < (int)vars.size() && carry == true){
		carry = false;
		varValIdxs[pos]++;
		if(varValIdxs[pos] >= (int)vars[pos]->vals().size()){
			varValIdxs[pos] = 0;
			pos++;
			carry = true;
		}
	}
	if(pos == (int)vars.size()){
		return false;
	}
	else{
		return true;
	}
}

//template<class T>
//bool Pgm::incVarValIdxs(std::vector<int>& varValIdxs,
//					const std::vector<std::vector<T> >& vals)
//{
//	//iterate through all combinations of variables
//	int pos = 0;
//	bool carry = true;
//	while(pos < vals.size() && carry == true){
//		carry = false;
//		varValIdxs[pos]++;
//		if(varValIdxs[pos] >= vals[pos].size()){
//			varValIdxs[pos] = 0;
//			pos++;
//			carry = true;
//		}
//	}
//	if(pos == vals.size()){
//		return false;
//	}
//	else{
//		return true;
//	}
//}

void Pgm::deleteContents(){
	for(int c = 0; c < (int)clustersData.size(); ++c){
		delete clustersData[c];
	}
//	for(int f = 0; f < (int)featsData.size(); f++){
//		delete featsData[f];
//	}
	for(int rv = 0; rv < (int)randVarsData.size(); rv++){
		delete randVarsData[rv];
	}
}


