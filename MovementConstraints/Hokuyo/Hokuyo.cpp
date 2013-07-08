/*
 * Hokuyo.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>
#include <opencv2/opencv.hpp>
#include "Hokuyo.h"

using namespace std;
using namespace cv;

Hokuyo::Hokuyo() {

}

Hokuyo::~Hokuyo() {

}

void Hokuyo::openPort(std::string port){
	urg_open(&hokuyo, URG_SERIAL, port.c_str(), 115200);
	urg_start_measurement(&hokuyo, URG_DISTANCE, URG_SCAN_INFINITY, 0);
}

void Hokuyo::closePort(){
	urg_stop_measurement(&hokuyo);
	urg_close(&hokuyo);
}

bool Hokuyo::isOpen(){
	return hokuyo.is_active;
}

//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
cv::Mat Hokuyo:: getData(){
	long data[HOKUYO_SCANS];
	long time;
	Mat ret(2, HOKUYO_SCANS, CV_32SC1);
	urg_get_distance(&hokuyo, data, &time);
	for(int i = 0; i < HOKUYO_SCANS; i++){
		double angle = urg_index2rad(&hokuyo, i);
		ret.at<int>(0, i) = data[i]*cos(angle);
		ret.at<int>(1, i) = data[i]*sin(angle);
	}
	return ret;
}
