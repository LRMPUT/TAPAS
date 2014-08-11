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
	float histHRangeMin, histHRangeMax, histSRangeMin, histSRangeMax, histVRangeMin, histVRangeMax;
	int histDLen, histILen;
	float histDRangeMin, histDRangeMax, histIRangeMin, histIRangeMax;
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

extern "C" void extractEntries(const unsigned char* const imageH,
								const unsigned char* const imageS,
								const unsigned char* const imageV,
								const float* const terrain,
								const int* const regionsOnImage,
								float* const feat,
								unsigned int* countPixelsEntries,
								unsigned int* countPointsEntries,
								const float* const cameraMatrix,
								const float* const distCoeffs,
								int numRows,
								int numCols,
								int numPoints,
								int numEntries,
								int descLen,
								const FeatParams* const featParams);

#endif /* CAMERACUDA_H_ */
