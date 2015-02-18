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

#include "ExtendedKalmanFilter.h"

#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter(float _Q, float _Rgps, float _Rimu, float _Renc, float _dt) {
	this->Q = cv::Mat::eye(4, 4, CV_64F) * _Q;

	this->Rgps = cv::Mat::eye(2, 2, CV_64F) * _Rgps;
	this->Rimu = cv::Mat::eye(1, 1, CV_64F) * _Rimu;
	this->Renc = cv::Mat::eye(1, 1, CV_64F) * _Renc;

	this->I = cv::Mat::eye(4, 4, CV_64F);
	this->dt = _dt;
	this->x_apriori = cv::Mat::zeros(4, 1, CV_64F);
	this->x_aposteriori = cv::Mat::zeros(4, 1, CV_64F);
	this->P_apriori = cv::Mat::zeros(4, 4, CV_64F);
	this->P_aposteriori = cv::Mat::zeros(4, 4, CV_64F);

	this->Hgps = cv::Mat::zeros(2, 4, CV_64F);
	this->Hgps.at<double>(0, 0) = 1.0;
	this->Hgps.at<double>(1, 1) = 1.0;

	this->Himu = cv::Mat::zeros(1, 4, CV_64F);
	this->Himu.at<double>(0, 3) = 1.0;

	this->Henc = cv::Mat::zeros(1, 4, CV_64F);
	this->Henc.at<double>(0, 2) = 1.0;
}

cv::Mat ExtendedKalmanFilter::state() {

	cv::Mat A = cv::Mat::zeros(4, 4, CV_64F);

	// 1st row
	A.at<double>(0, 0) = 1.0;
	A.at<double>(0, 2) = cos(this->x_aposteriori.at<double>(3)) * this->dt;

	// 2nd row
	A.at<double>(1, 1) = 1;
	A.at<double>(1, 2) = sin(this->x_aposteriori.at<double>(3)) * this->dt;

	// 3rd row
	A.at<double>(2, 2) = 1;

	// 4th row
	A.at<double>(3, 3) = 1;


	return A;
}

cv::Mat ExtendedKalmanFilter::jacobian() {

	cv::Mat F = cv::Mat::zeros(4, 4, CV_64F);

	// 1st row
	F.at<double>(0, 0) = 1.0;
	F.at<double>(0, 2) = cos(this->x_aposteriori.at<double>(3)) * this->dt;
	F.at<double>(0, 3) = -sin(this->x_aposteriori.at<double>(3)) * this->dt;

	// 2nd row
	F.at<double>(1, 1) = 1.0;
	F.at<double>(1, 2) = sin(this->x_aposteriori.at<double>(3)) * this->dt;
	F.at<double>(1, 3) = cos(this->x_aposteriori.at<double>(3)) * this->dt;

	// 3rd row
	F.at<double>(2, 2) = 1;

	// 4th row
	F.at<double>(3, 3) = 1;

	return F;
}

void ExtendedKalmanFilter::predict(float _dt) {
	this->dt = _dt;
	this->x_apriori = this->state() * this->x_aposteriori;

	cv::Mat F = this->jacobian();
	this->P_apriori = F * this->P_aposteriori * F.t() + this->Q;
}


cv::Mat ExtendedKalmanFilter::correctGPS(cv::Mat gpsMeasurement) {
	cv::Mat K = cv::Mat::zeros(4, 2, CV_64F);
	K = (this->P_apriori * this->Hgps.t())
			* (this->Hgps * this->P_apriori * this->Hgps.t() + this->Rgps).inv();
	this->x_aposteriori = this->x_apriori
			+ K * (gpsMeasurement - this->Hgps * this->x_apriori);
	this->P_aposteriori = (this->I - K * this->Hgps) * this->P_apriori;
	return this->x_aposteriori;
}


cv::Mat ExtendedKalmanFilter::correctIMU(cv::Mat imuMeasurement) {
	cv::Mat K = cv::Mat::zeros(4, 1, CV_64F);
	K = (this->P_apriori * this->Himu.t())
			* (this->Himu * this->P_apriori * this->Himu.t() + this->Rimu).inv();

	this->x_aposteriori = this->x_apriori
			+ K * (imuMeasurement - this->Himu * this->x_apriori);
	this->P_aposteriori = (this->I - K * this->Himu) * this->P_apriori;
	return this->x_aposteriori;
}


cv::Mat ExtendedKalmanFilter::correctEncoder(cv::Mat encMeasurement) {
	cv::Mat K = cv::Mat::zeros(4, 1, CV_64F);
	K = (this->P_apriori * this->Henc.t())
			* (this->Henc * this->P_apriori * this->Henc.t() + this->Renc).inv();
	this->x_aposteriori = this->x_apriori
			+ K * (encMeasurement - this->Henc * this->x_apriori);
	this->P_aposteriori = (this->I - K * this->Henc) * this->P_apriori;
	return this->x_aposteriori;
}

