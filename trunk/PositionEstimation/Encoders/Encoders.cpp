/*
 * Encoders.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: smi
 */

#include "Encoders.h"
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace cv;
using namespace trobot;

Encoders::Encoders() {

}

Encoders::Encoders(const std::string& device, unsigned int baud){
	openPort(device, baud);
}

Encoders::~Encoders() {
	closePort();
}

void Encoders::openPort(const std::string& device, unsigned int baud){
	serialPort.open(baud, device);
}

void Encoders::closePort(){
	serialPort.close();
}

bool Encoders::isPortOpen(){
	return serialPort.isActive();
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS

//CV_32SC1 2x1: left, right encoder
cv::Mat Encoders::getEncoders(){
	Mat ret(2, 1, CV_32SC1);
	serialPort.startReadCount();
	serialPort.write("req");
	for(int i = 0; i < 50; i++){
		if(serialPort.getReadCount() >= 22){
			break;
		}
		usleep(1000);
	}
	boost::circular_buffer<char> data = serialPort.getDataRead();
	int pos = searchBufferR(data, "E ");
	static const int bufferLen = 40;
	char buffer[bufferLen];
	if(pos >= 0){
		for(int i = pos; i < min((int)data.size(), bufferLen - 1); i++){
			buffer[i - pos] = data[i];
		}
		buffer[min((int)data.size(), bufferLen -1)] = 0;
	}
	int left, right;
	sscanf(buffer, "E %d %d", &left, &right);
	ret.at<int>(0) = left;
	ret.at<int>(1) = right;
	return ret;
}
