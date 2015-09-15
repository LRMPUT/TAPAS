/*
 * inference.h
 *
 *  Created on: 16 gru 2014
 *      Author: jachu
 */

#ifndef INFERENCE_H_
#define INFERENCE_H_

#include <vector>

#include "../Pgm/Pgm.h"

class Inference{

public:
	struct InfEdge{
		int i, j;
		int w;
		InfEdge() {}
		InfEdge(int ii, int ij, int iw) : i(ii), j(ij), w(iw) {}
	};

private:

	//std::vector<double> marginalize(Cluster* cluster,
	//								const std::vector<RandVar*>& vars);

	/*std::vector<double> compSumProdMsg(Cluster* src,
										Cluster* dst,
										const std::vector<std::vector<double> >& inMsgs);

	std::vector<double> compMaxSumMsg(Cluster* src,
										Cluster* dst,
										const std::vector<std::vector<double> >& inMsgs);*/

	static std::vector<double> compCurMarginal(Cluster* clust,
										MargType type,
										const std::vector<std::vector<double> >& inMsgs,
										const std::vector<double>& params,
										const std::vector<double>& obsVec = std::vector<double>());

	static std::vector<int> selectMST(const std::vector<InfEdge>& edges, //weight sorted
								int numClusters);

	static void passMessage(const Pgm& pgm,
						MargType type,
						std::vector<std::vector<std::vector<double> > >& msgs,
						std::vector<std::vector<std::vector<double> > >& prevMsgs,
						Cluster* src,
						Cluster* dst,
						const std::vector<double>& params,
						const std::vector<double>& obsVec = std::vector<double>());

	static void treeReparametrize(const Pgm& pgm,
							MargType type,
							std::vector<std::vector<std::vector<double> > >& msgs,
							std::vector<std::vector<std::vector<double> > >& prevMsgs,
							const std::vector<InfEdge>& edges,
							const std::vector<int>& selEdges,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());

	static bool runBP(const Pgm& pgm,
						MargType type,
						std::vector<std::vector<std::vector<double> > >& msgs,
						const std::vector<double>& params,
						const std::vector<double>& obsVec = std::vector<double>());

public:
	static std::vector<std::vector<double> > compMarginalsBF(const Pgm& pgm,
															double& logPartFunc,
															const std::vector<double>& params,
															const std::vector<double>& obsVec = std::vector<double>());

	static double compJointProbUnnorm(const Pgm& pgm,
									const std::vector<double>& vals,
									const std::vector<double>& params,
									const std::vector<double>& obsVec = std::vector<double>());

	static std::vector<std::vector<double> > compMarginalsParam(const Pgm& pgm,
													double& logPartFunc,
													std::vector<std::vector<std::vector<double> > >& msgs,
													const std::vector<double>& params,
													const std::vector<double>& obsVec = std::vector<double>());

	static std::vector<std::vector<double> > compMarginals(const Pgm& pgm,
													double& logPartFunc,
													std::vector<std::vector<std::vector<double> > >& msgs,
													const std::vector<double>& obsVec = std::vector<double>());

	static bool compMAPParam(const Pgm& pgm,
							std::vector<std::vector<double> >& marg,
							std::vector<std::vector<std::vector<double> > >& msgs,
							const std::vector<double>& params,
							const std::vector<double>& obsVec = std::vector<double>());
};



#endif /* INFERENCE_H_ */
