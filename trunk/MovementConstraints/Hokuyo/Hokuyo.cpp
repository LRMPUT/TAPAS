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
	hokuyo.open(port.c_str(), qrk::Urg_driver::Default_baudrate, qrk::Urg_driver::Serial);

	//cout << hokuyo.deg2index(-135) << " " << hokuyo.deg2index(135) << endl;
	cout << hokuyo.set_scanning_parameter(hokuyo.deg2index(-135), hokuyo.deg2index(135)) << endl;
}

void Hokuyo::closePort(){
	cout << "Closing hokuyo" << endl;
	//hokuyo.close();
}

bool Hokuyo::isOpen(){
	return hokuyo.is_open();
}

//CV_32SC1 3xHOKUYO_SCANS: x, y points from left to right
const cv::Mat Hokuyo:: getData(){
	vector<long int> length;
	vector<unsigned short> intensity;
	Mat ret(3, HOKUYO_SCANS, CV_32SC1);

	hokuyo.start_measurement(qrk::Urg_driver::Distance, 1, 0);
	hokuyo.get_distance(length);
	for(int i = 0; i < length.size(); i++){
		double angle = hokuyo.index2rad(i);
		//cout << "Point " << i << " = " << length[i] << endl;
		//cout << "Point " << i << " = (" << data[i]*cos(angle) << ", " << data[i]*sin(angle) << ")" << endl;
		ret.at<int>(0, i) = length[i]*cos(angle);
		ret.at<int>(1, i) = length[i]*sin(angle);
		ret.at<int>(2, i) = length[i];
	}
	return ret;
}
