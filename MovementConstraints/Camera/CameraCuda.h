/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

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
								int descLen,
								const FeatParams* const featParams);

#endif /* CAMERACUDA_H_ */
