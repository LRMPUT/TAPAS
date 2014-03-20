/*
 * Inference.h
 *
 *  Created on: 20-03-2014
 *      Author: jachu
 */

#ifndef INFERENCE_H_
#define INFERENCE_H_

enum InferenceType{
	monteCarlo,
	beliefPropagation
};

class Inference{

public:
	Inference();
	double computeMarginal(const Graph& graph, int node);
	double computeNormConst(const Graph& graph);
};


#endif /* INFERENCE_H_ */
