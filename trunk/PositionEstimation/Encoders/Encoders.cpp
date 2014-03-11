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
	static boost::circular_buffer<char> data(50);
	int left, right;
	bool readEncoders = false;
	while(!readEncoders){
		boost::circular_buffer<char> newData = serialPort.getDataRead();
		//cout << "newData:" << endl;
		for(int i = 0; i < newData.size(); i++){
			//cout << newData[i];
			data.push_back(newData[i]);
		}
		//cout << endl;
		/*cout << "data:" << endl;
		for(int i = 0; i < data.size(); i++){
			cout << data[i];
		}
		cout << endl;*/
		int posBeg = searchBufferR(data, "E ");
		int posEnd = searchBufferR(data, "_");
		//cout << "posBeg = " << posBeg << ", posEnd = " << posEnd << endl;
		static const int bufferLen = 40;
		char buffer[bufferLen];
		if(posBeg >= 0){
			for(int i = 0; i < posBeg; i++){
				data.pop_front();
			}
			if(posBeg < posEnd && posEnd >= 0){
				for(int i = posBeg; i < posEnd; i++){
					buffer[i - posBeg] = data.front();
					data.pop_front();
				}
				buffer[posEnd - posBeg] = 0;
				sscanf(buffer, "E %d %d", &left, &right);
				readEncoders = true;
			}
		}
		usleep(1000);
	}
	ret.at<int>(0) = left;
	ret.at<int>(1) = right;
	return ret;
}
