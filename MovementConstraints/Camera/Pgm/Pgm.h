/*
 * pgm.h
 *
 *  Created on: 16 gru 2014
 *      Author: jachu
 */

#ifndef PGM_H_
#define PGM_H_

#include <vector>
#include <ostream>

extern std::vector<double> featScales;

template<class T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& vec){
	out << "[";
	for(int v = 0; v < (int)vec.size(); ++v){
		out << vec[v];
		if(v < vec.size() - 1){
			out << ", ";
		}
	}
	out << "]";

	return out;
}

enum MargType{
	SumProduct,
	MaxSum
};


class RandVar{
protected:
	int idData;
	bool isObserved;
	//double observedVal;
	std::vector<double> valsData;
public:
	RandVar(int iid, std::vector<double> ivals);

	RandVar(int iid, double observedVal);

	virtual ~RandVar() {}

	inline int id() const {
		return idData;
	}

	inline bool isObs() const {
		return isObserved;
	}

	void makeObserved(double val);

	void makeNotObserved(std::vector<double> vals);

	inline const std::vector<double>& vals() const{
		return valsData;
	}

	void setVals(const std::vector<double>& newVals);
};

class Feature{
	int paramNumData;
	int idData;
	std::vector<int> obsNumsData;
public:
	Feature(int iid, int iparamNum, const std::vector<int>& iobsNums = std::vector<int>());
	virtual ~Feature();

	inline int paramNum() const {
		return paramNumData;
	}

	inline const std::vector<int>& obsNums(){
		return obsNumsData;
	}

	inline int id() const {
		return idData;
	}

	virtual double comp(const std::vector<double>& vals,
						const std::vector<double>& obsVec = std::vector<double>()) = 0;

	virtual double compParam(const std::vector<double>& vals,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>()) = 0;
};

/*class Marginal{
	int dim;
	std::vector<int> numVals;
	std::vector<double> valData;
public:
	Marginal(int idim, const std::vector<int> inumVals);
	inline double& val(const std::vector<int>& idxs){
		int pos = 0;
		for(int d = 0; d < dim; ++d){

		}
	}
};*/

class Cluster{
protected:
	int idData;
	std::vector<Cluster*> nhData;	//id sorted
	std::vector<std::vector<RandVar*> > sepsetsData;	//id sorted
	std::vector<Feature*> featuresData;
	std::vector<RandVar*> randVarsData;	//id sorted
	//const std::vector<double>* paramsData;
	std::vector<int> randVarsOrderData;
	std::vector<int> obsVecIdxsData;

	void normalizeMarg(std::vector<double>& marg,
						MargType type) const;
public:
	Cluster(int iid,
			const std::vector<Feature*>& ifeats,
			const std::vector<RandVar*>& irandVars /*id sorted*/,
			const std::vector<int>& irandVarsOrder,
			const std::vector<int>& iobsVecIdxs = std::vector<int>());

	virtual ~Cluster(){}

	void setNh(std::vector<Cluster*> newNh /*id sorted*/);

	void setSepsets(std::vector<std::vector<RandVar*> > newSepsets /*id sorted*/);

	inline int id() const {
		return idData;
	}

	virtual double compFactorsVal(const std::vector<double>& varVals,
								MargType type,
								const std::vector<double>& params,
								const std::vector<double>& obsVec = std::vector<double>()) const;

	virtual double compSumHcdiv(const std::vector<std::vector<double> >& inMsgs,
								const std::vector<double>& params,
								const std::vector<double>& obsVec);

	virtual std::vector<double> compSumModelExpectation(const std::vector<std::vector<double> >& inMsgs,
														const std::vector<double>& params,
														const std::vector<double>& obsVec);

	virtual std::vector<double> compSumEmpiricalExpectation(const std::vector<double>& varVals,
														const std::vector<double>& obsVec) const;

	virtual std::vector<std::vector<int> > getBestValsIdxs(const std::vector<std::vector<double> >& inMsgs,
															const std::vector<double>& params,
															const std::vector<double>& obsVec,
															double eps);

	virtual std::vector<double> marginalize(const std::vector<RandVar*>& margVars /*id sorted*/,
									MargType type,
									const std::vector<std::vector<double> >& inMsgs,
									const std::vector<double>& params,
									const std::vector<double>& obsVec = std::vector<double>(),
									const Cluster* excluded = 0);

	inline const std::vector<Cluster*>& nh() const{
		return nhData;
	}

	inline const std::vector<Feature*>& feats() const{
		return featuresData;
	}

	inline const std::vector<RandVar*>& randVars() const{
		return randVarsData;
	}

	inline const std::vector<RandVar*>& sepset(int nhc /*nh index*/) const{
		return sepsetsData[nhc];
	}

	inline const std::vector<std::vector<RandVar*> >& sepsets() const{
		return sepsetsData;
	}

	inline const std::vector<int>& obsVecIdxs() const{
		return obsVecIdxsData;
	}
	inline std::vector<double> getCurObsVec(const std::vector<double> obsVec) const {
		std::vector<double> curObsVec;
		if(!obsVec.empty()){
			for(int o = 0; o < (int)obsVecIdxsData.size(); ++o){
				curObsVec.push_back(obsVec[obsVecIdxsData[o]]);
			}
		}
		return curObsVec;
	}
};

class Pgm{
	std::vector<RandVar*> randVarsData;	//id (0, .., n-1) sorted
	std::vector<Cluster*> clustersData;	//id (0, .., m-1) sorted
	std::vector<Feature*> featsData;
	std::vector<double> paramsData;
public:
	Pgm() {}

	Pgm(std::vector<RandVar*> irandVars,
		std::vector<Cluster*> iclusters,
		std::vector<Feature*> ifeats);

	static bool incVarValIdxs(std::vector<int>& varValIdxs,
						const std::vector<RandVar*>& vars);

	template<class T>
	static bool incVarValIdxs(std::vector<int>& varValIdxs,
						const std::vector<std::vector<T> >& vals)
	{
		//iterate through all combinations of variables
		int pos = 0;
		bool carry = true;
		while(pos < vals.size() && carry == true){
			carry = false;
			varValIdxs[pos]++;
			if(varValIdxs[pos] >= vals[pos].size()){
				varValIdxs[pos] = 0;
				pos++;
				carry = true;
			}
		}
		if(pos == vals.size()){
			return false;
		}
		else{
			return true;
		}
	}

	static inline int roundToInt(double val){
		return (int)(val + (val >= 0.0 ? 0.5 : -0.5));	//round to nearest int
	}

	void deleteContents();

	inline const std::vector<RandVar*>& constRandVars() const{
		return randVarsData;
	}

	inline std::vector<RandVar*>& randVars(){
		return randVarsData;
	}

	inline const std::vector<Cluster*>& constClusters() const{
		return clustersData;
	}

	inline std::vector<Cluster*>& clusters(){
		return clustersData;
	}

	inline std::vector<Feature*>& feats(){
		return featsData;
	}

	inline const std::vector<double>& constParams() const{
		return paramsData;
	}

	inline std::vector<double>& params(){
		return paramsData;
	}
};

inline bool compIdRandVars(const RandVar* lh, const RandVar* rh){
	return lh->id() < rh->id();
}

inline bool compIdFeats(const Feature* lh, const Feature* rh){
	return lh->id() < rh->id();
}

inline bool compIdClust(const Cluster* lh, const Cluster* rh){
	return lh->id() < rh->id();
}

inline bool compIdPairClustPInt(const std::pair<Cluster*, int>& lh, const std::pair<Cluster*, int>& rh){
	return lh.first->id() < rh.first->id();
}

#endif /* PGM_H_ */
