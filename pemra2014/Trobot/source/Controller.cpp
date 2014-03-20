#include <iostream>
#include <cmath>
#include <string>
#include <boost/circular_buffer.hpp>
#include "../include/SerialPort.h"
#include "../include/Controller.h"

using namespace std;
using namespace boost;

namespace trobot {
	/*Controller::Controller() //:
	//serialPort_( 9600, "COM20")

	{

	}*/

	Controller::Controller(unsigned int baud, const std::string& device) :
voltsPerUnit(0.0049)
{			
	baud_ = baud;
	device_ = device;
	try {
		serialPort_ = new SerialPort(baud, device);
	}
	catch (std::exception e) {
		//delete [] serialPort_;
		searchForDevice();
		return;
	}
	if(!connectionTestOk())
		searchForDevice();
}




Controller::~Controller() {

}

unsigned int Controller::getAnalogReading(unsigned int pinNo) {
	if (pinNo < nAnalogPins_) {
		if( serialPort_->newDataAvailable() ) 
			processFrame();
		return analogReading_[pinNo];
	}
	else throw "Bad pin number";
}

bool Controller::getPinState(unsigned int pinNo) {
	if (pinNo < nDigitalPins_) {
		if( serialPort_->newDataAvailable() ) 
			processFrame();
		return pinState_[ pinNo ];
	}
	else throw "Bad pin number";
}

pinModeType Controller::getPinMode(unsigned int pinNo) {
	if (pinNo < nDigitalPins_) {
		if( serialPort_->newDataAvailable() ) 
			processFrame();
		return pinMode_[ pinNo ];
	}
	else throw "Bad pin number";
}

void Controller::setPinMode(unsigned int pinNo, trobot::pinModeType pinMode) {
	if (pinNo < nDigitalPins_)
		pinModeToWrite_[ pinNo ] = pinMode;
	else throw "Bad pin number";
}

void Controller::processFrame() {




	// copying data from buffer, so they don't change while we process them
	const circular_buffer<char>  data = serialPort_->getDataRead();


	bool frameProcessed = false;
	int i = data.size() - 1;

	while( (!frameProcessed) && (i > 1)) {
		// looking for last frame ending
		//cout << data.at(i);

		if(data[ i ] == stopChar) 
		{

			int j = i - (frameLength - 1);
			char * dataCh = new char[frameLength];

			// checking frame beginning
			if (data[ j ] != startChar)	
			{
				//data.clear();
				serialPort_->flush();
				cout << "Controller: Frame processing error" << endl;
				return;
				//throw std::exception("Haven't recieved any complete frame",0);
			}
				

			else {
				j++;
				for(int k = 0; k < frameLength - 2; k++) {// minus stop and start bytes
					dataCh[ k ] = data[ k  + j ];
				}


				for(int k = 0; k < nAnalogPins_; k++) { // reading analog pins						
					analogReading_[ k ] = hex2int(&dataCh[ k * 4 ]);
				}
				// reading digital pins	
				int digitalReadingTemp = hex2int(&dataCh[ nAnalogPins_ * 4 ]);
				for(int k = 0; k < nDigitalPins_; k++) {
					pinState_[ k ] = (digitalReadingTemp >> k) & 1;
				}

				// reading pin mode
				int pinModeTemp = hex2int(&dataCh[ (nAnalogPins_ + 1) * 4 ]);
				for(int k = 0; k < nDigitalPins_; k++) {
					if((( pinModeTemp >> k) & 1) == 1)
						pinMode_[ k ] = modeOutput;
					else pinMode_[ k ] = modeInput;
				}
				//cout << "frame ok\n";
				
				frameProcessed = true;

			}
			delete[] dataCh ;
		}
		else i--;
	}



}




void Controller::setPinState(unsigned int pinNo, bool state) {
	if (pinNo < nDigitalPins_)
		pinStateToWrite_[ pinNo ] = state;
	else throw "Bad pin number";
}

void Controller::sendData() {
	//serialPort_->write(specialChar);
	serialPort_->write(startChar);

	writePinMode(pinModeToWrite_, nDigitalPins_);
	writeBoolArray(pinStateToWrite_, nDigitalPins_);


	//serialPort_->write(specialChar);
	serialPort_->write(stopChar);
}

void Controller::writeInt16(int data) {
	char temp[4];
	sprintf(temp,"%4X",data);
	serialPort_->write(temp);
}

void Controller::writePinMode(trobot::pinModeType *mode, int length) {
	char temp = 0;
	for (int i = 0; i < length; i++) {
		if(mode[ i ] == true)
			temp |= (1 << (i % 16));
		
		if ((i > 0) && (((i + 1) % 16 == 0) || (i == length - 1))) {
			char toSend[4];
			sprintf(toSend,"%4X",temp);
			serialPort_->write(toSend);
			temp = 0;
		}
	}
}

void Controller::writeBoolArray(bool * bools, int length) {
	char temp = 0;
	for (int i = 0; i < length; i++) {
		if(bools[ i ] == true)
			temp |= (1 << (i % 16));
		
		if ((i > 0) && (((i + 1) % 16 == 0) || (i == length - 1))) {
			char toSend[4];
			sprintf(toSend,"%4X",temp);
			serialPort_->write(toSend);
			temp = 0;
		}
	}
}

bool Controller::connectionTestOk() {
	sleep(2); // waiting for arduino to boot
	serialPort_->flush();
	serialPort_->write("#S");
	int i = 0;
	int timeoutCount = 10;
		do
		{
			sleep(1);
			i++;
		}while( (i < timeoutCount) && (!serialPort_->newDataAvailable()));
		

	circular_buffer<char> data = serialPort_->getDataRead();


	int result = searchBufferR(data, "trobot"); //looking for a identification string
	if (result >= 0) {
		cout << "Controller connected to: " << device_ << endl;
		return true;
	}
	else {
		serialPort_->close();
		return false;
	}

}

void Controller::searchForDevice() {
	vector<string> comList;
	SerialPort::DetectComPorts(comList, 128);

	for(vector<string>::iterator port = comList.begin(); port != comList.end(); port++) {
		try {
			serialPort_ = new SerialPort(baud_, *port);
		}
		catch(...) {
			cout << "Controller: connection to " << *port << " did not succeed. "<< endl;
			continue;
		}
		device_ = *port;
		if( connectionTestOk() )
			return;
		else
			cout << "Controller: connection to " << *port << " did not succeed. "<< endl;
	}

}

unsigned int Controller::hex2int(char * hex) {
	unsigned int result = 0;
	for(int i = 0; i < 4; i++) {
		if(hex[i] > 47 && hex[i] < 58) {			
			result = result + pow((float)16,(3-i)) * (hex[i] - 48);
		}
		else if(hex[i] > 64 && hex[i] < 71) {
			result = result + pow((float)16,(3-i)) * (hex[i] - 55);
		}
		else if(hex[i] > 96 && hex[i] < 103) {
			result = result + pow((float)16,(3-i)) * (hex[i] - 87);
		}

	}
	return result;
}

}
