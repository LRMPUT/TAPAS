/*
 * Hokuyo.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include <urg_cpp/Urg_driver.h>
#include <opencv2/opencv.hpp>
#include "Hokuyo.h"

using namespace std;
using namespace cv;

Hokuyo::Hokuyo() {

}

Hokuyo::~Hokuyo() {

}

void Hokuyo::openPort(std::string port){
	cout << hokuyo.open(port.c_str(), qrk::Urg_driver::Default_baudrate, qrk::Urg_driver::Serial) << endl;

	//cout << hokuyo.product_type() << endl;
	//cout << hokuyo.deg2index(-135) << " " << hokuyo.deg2index(135) << endl;
	hokuyo.set_scanning_parameter(hokuyo.deg2index(-135), hokuyo.deg2index(135));
}

void Hokuyo::closePort(){
	cout << "Closing hokuyo" << endl;
	//hokuyo.close();
}

bool Hokuyo::isOpen(){
	return hokuyo.is_open();
}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat Hokuyo:: getData(){
	vector<long int> length;
	vector<unsigned short> intensity;
	Mat ret(4, HOKUYO_SCANS, CV_32SC1);

	hokuyo.start_measurement(qrk::Urg_driver::Distance_intensity, 1, 0);
	hokuyo.get_distance_intensity(length, intensity);
	for(int i = 0; i < length.size(); i++){
		double angle = hokuyo.index2rad(i);
		//cout << "Point " << i << " = " << length[i] << endl;
		//cout << "Point " << i << " = (" << data[i]*cos(angle) << ", " << data[i]*sin(angle) << ")" << endl;
		ret.at<int>(0, i) = length[i]*cos(angle);
		ret.at<int>(1, i) = length[i]*sin(angle);
		ret.at<int>(2, i) = length[i];
		ret.at<int>(3, i) = intensity[i];
	}
	return ret;
}
