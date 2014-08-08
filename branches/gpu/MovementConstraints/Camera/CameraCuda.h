/*
 * CameraCuda.h
 *
 *  Created on: Aug 8, 2014
 *      Author: robots
 */

#ifndef CAMERACUDA_H_
#define CAMERACUDA_H_

struct FeatParams{
	int histHLen, histSLen, histVLen;
	int histHRangeMin, histHRangeMax, histSRangeMin, histSRangeMax, histVRangeMin, histVRangeMax;
	int histDLen, histILen;
	int histDRangeMin, histDRangeMax, histIRangeMin, histIRangeMax;
};

extern "C" void reprojectCameraPoints(float* invCameraMatrix,
		float* distCoeffs,
		float* curPosCameraMapCenterGlobal,
		float* curPosCameraMapCenterImu,
		int numRows,
		int numCols,
		int* segments,
		int mapSize,
		int rasterSize);


#endif /* CAMERACUDA_H_ */
