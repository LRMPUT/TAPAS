/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
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

#include <opencv2/core/core.hpp>
#include <cmath>

class ExtendedKalmanFilter {

	cv::Mat Rgps, Rimu, Renc, Q; // Covariances of correct/predict

	cv::Mat I; // Identity
	float dt;
	cv::Mat x_apriori, x_aposteriori; // 4x1 matrices
	cv::Mat P_apriori, P_aposteriori; // 4x4 matrices
	cv::Mat Hgps, Himu, Henc;

public:
	ExtendedKalmanFilter(float _Q = 10, float _Rgps = 100, float _Rimu = 1/180*M_PI, float _Renc = 0.01, float _dt = 0.01);

	void predict(float dt);

	cv::Mat correctGPS(cv::Mat gpsMeasurement);
	cv::Mat correctIMU(cv::Mat imuMeasurement);
	cv::Mat correctEncoder(cv::Mat encMeasurement);

private:
	// TODO: Fill jacobian with real values
	cv::Mat jacobian();
	// TODO: Check state implementation in MATLAB
	cv::Mat state();
};
