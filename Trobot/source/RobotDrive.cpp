#include "../include/RobotDrive.h"
#include "../include/RoboteqDevice.h"
#include "../include/SerialPort.h"
#include <boost/circular_buffer.hpp>
#include <string>

using namespace std;
using namespace boost;

namespace trobot {
	RobotDrive::RobotDrive(const string& device, unsigned int baud)

	{
		//cout << "RobotDrive::RobotDrive" << endl;
		baud_ = baud;
		device_ = device;

		try {
			cout << "Opening serial port " << device << endl;
			serialPort_ = new SerialPort(baud, device);
		}
		catch (...) {
			//cout << "Caught exception" << endl;
			std::vector<std::string> comList;
			serialPort_->DetectComPorts(comList, 128);
			searchForDevice();
		}
		if(!connectionTestOk()){
			serialPort_->close();
			delete serialPort_;
//			searchForDevice();
			cout << "Roboteq driver NOT connected!\n";
		}
		else{
			cout << "Roboteq driver connected!\n";
		}

		encoderCPR = 48 * 75; //encoder CPR * gear ratio
		//encoderCPR = 64 * 122; //encoder CPR * gear ratio
		wheelDiameter = 0.12;
		maxRpms = 115;
		distanceBetweenWheels = 0.24;
		lastCommand = STOP;

		positionPid.kd = 5;
		positionPid.ki = 0;
		positionPid.kp = 20;

		speedPid.kp = 20;
		speedPid.kd = 5;
		speedPid.ki = 40;

		mode_ = -1;

		initConfig();

	}

	RobotDrive::~RobotDrive(void)
	{
		serialPort_->close();
		delete serialPort_;
	}

	void RobotDrive::runMotor(int speed, int channel) {
		//setting mode to position
		setPositionMode();

		// set the speed
		int actualRpms = (int)(maxRpms *  abs(speed) / 1000);

		setConfig("MVEL",channel,actualRpms);

		if(speed < 0) {
			setRuntime("P",channel, -50000000);
		}
		else
			setRuntime("P",channel, 50000000);
	}


	void RobotDrive::stop() {
		if( mode_ == SPEED) {
				setRuntime("M", 0, 0);
				lastCommand = STOP;
		}
		if ( mode_ == POSITION) {
			setConfig("MVEL",1,0);
			setConfig("MVEL",2,0);
			//driveStraight(200, 0);
			//lastCommand = STOP;
		}
	}


	void RobotDrive::driveStraight(int speed) {
		//setPositionMode();
		if (speed < 0) {
			speed = -speed;
			driveStraight(speed, -50000);
		}
		else {
			driveStraight(speed, 50000);
		}
	}





	void RobotDrive::driveStraight(int speed, float distance) {

		//setting mode to position
		setPositionMode();

		// set the speed
		int actualRpms = (int)(maxRpms *  abs(speed) / 1000);

		setConfig("MVEL",1,20);
		setConfig("MVEL",2,20);

		int encoder1 = 0;
		int encoder2 = 0;

		getRuntime("C", &encoder1, &encoder2);

		int countToGo1 = (int)(distance / (wheelDiameter * 3.14) * encoderCPR) + encoder1;
		int countToGo2 = (int)(distance / (wheelDiameter * 3.14) * encoderCPR) + encoder2;


		setRuntime("P",1, countToGo1);
		setRuntime("P",2, countToGo2);

		lastGoalL = countToGo1;
		lastGoalR = countToGo2;
		lastSpeed = speed;
		lastCommand = POSITION;

	}

	void RobotDrive::getEncoder(int* enc1, int* enc2){
		getRuntime("C", enc1, enc2);
	}

	void RobotDrive::initConfig() {
		// disable echo
		setConfig("ECHOF", 1, 1);

		// encoders configured as feedbacks
		setConfig("EMOD", 1, 18);
		setConfig("EMOD", 2, 34);

		// setting encoder pulse per revolution
		setConfig("EPPR", 1, encoderCPR / 4);
		setConfig("EPPR", 2, encoderCPR / 4);


		// disable watchdog
		setConfig("RWD",0,0);



        // set accelleration and decelleration
		setRuntime("AC",1,120);
		setRuntime("AC",2,120);

		setRuntime("DC",1,120);
		setRuntime("DC",2,120);

		// disable integral tracking error
		setConfig("CLERD", 1, 0);
		setConfig("CLERD", 2, 0);

		setPositionMode();

	}

	void RobotDrive::turn(int speed, int degrees) {
		//setting mode to position
		setPositionMode();

		// set the speed
		int actualRpms = (int)(maxRpms *  abs(speed) / 1000);

		setConfig("MVEL",1,actualRpms);
		setConfig("MVEL",2,actualRpms);

		float distance = (float)degrees * 3.14 * distanceBetweenWheels / 360.0;

		int encoder1 = 0;
		int encoder2 = 0;

		getRuntime("C", &encoder1, &encoder2);

		int countToGo1 = (int)(distance / (wheelDiameter * 3.14) * encoderCPR) + encoder1;
		int countToGo2 = - (int)(distance / (wheelDiameter * 3.14) * encoderCPR) + encoder2;


		setRuntime("P",1, countToGo1);
		setRuntime("P",2, countToGo2);


		lastGoalL = countToGo1;
		lastGoalR = countToGo2;
		lastSpeed = speed;

		lastCommand = TURN;


	}


	bool RobotDrive::positionReached() {
		if (( lastCommand != TURN ) && ( lastCommand != POSITION ))
			return true;
		else
		{

			int s1, s2;
			getRuntime("S", 0, &s1, &s2);

			// these 'ifs' prevent strange errors that seem like some kind of overflow inside Roboteq driver
			if (s1 < -20000)
                {
                    s1 += 21846;
                }
                if (s2 < -20000)
                {
                    s2 += 21846;
                }
                if (s1 > 20000)
                {
                    s1 -= 21845;
                }
                if (s2 > 20000)
                {
                    s2 -= 21845;
                }


			if((s1 == 0) && (s2 == 0)) {
				if (posCheckedFlag_ == true) {
					posCheckedFlag_ = false;
					return true;
				}
				else {
					posCheckedFlag_ = true;
				}

			}

			getRuntime("S", 0, &s1, &s2);

			if((s1 == 0) && (s2 == 0)) {
				//make sure the reading was not fake
				getRuntime("S", 0, &s1, &s2);			
				if((s1 == 0) && (s2 == 0)) {
					return true;				
				}				
			}

		}
		return false;

	}


	void RobotDrive::setRuntime(std::string command, int param1, int param2) {
		stringstream sstream;
		string comm;

		sstream << "!" << command << " " << param1 << " " << param2 << "\r";
		comm = sstream.str();

		serialPort_->write(comm);

	}

	void RobotDrive::setConfig(std::string command, int param1, int param2) {
		stringstream sstream;
		string comm;

		sstream << "^" << command << " " << param1 << " " << param2 << "\r";
		comm = sstream.str();

		serialPort_->write(comm);

	}

	void RobotDrive::setPositionMode() {

		if(mode_ != POSITION) {
			stop();

			setRuntime("EX",0,0);
			cout << "emergency mode\n";

			int encoder1 = 0;
			int encoder2 = 0;

			getRuntime("C", &encoder1, &encoder2);


			setConfig("KP", 1, positionPid.kp);
			setConfig("KD", 1, positionPid.kd);
			setConfig("KI", 1, positionPid.ki);

			setConfig("KP", 2, positionPid.kp);
			setConfig("KD", 2, positionPid.kd);
			setConfig("KI", 2, positionPid.ki);

			cout << "Config set\n";
			usleep(50000);


			setConfig("MMOD", 1, 3);
			setConfig("MMOD", 2, 3);
			cout << "Position mode\n";
			usleep(50000);


			setRuntime("P",1, encoder1);
			setRuntime("P",2, encoder2);
			cout << "Position 0 again\n";



			mode_ = POSITION;
			usleep(500000);
			setRuntime("MG",0,0);
			cout << "quit emergency mode\n";
		}






	}


	/*void RobotDrive::setSpeedMode() {
		// currently controller gets out of controll when switching from speed to position mode. Issue reported to Roboteq

		if(mode_ != SPEED) {
			stop();

			setConfig("MMOD", 1, 0);
			setConfig("MMOD", 2, 0);

			setRuntime("M", 0, 0);

			setConfig("KP", 1, speedPid.kp);
			setConfig("KD", 1, speedPid.kd);
			setConfig("KI", 1, speedPid.ki);

			setConfig("KP", 2, speedPid.kp);
			setConfig("Kd", 2, speedPid.kd);
			setConfig("KI", 2, speedPid.ki);

			mode_ = SPEED;
		}





	}*/


	void RobotDrive::getRuntime(const std::string &command, int param, int *result1, int *result2) {
		stringstream sstream;
		stringstream sResult;
		string comm;

		if(param == NO_PARAM) {
			sstream << "?" << command << "\r";
		}
		else {		
			sstream << "?" << command << " " << param << "\r";
		}

		comm = sstream.str();

		serialPort_->write(comm);
		usleep(50000);
		
		// preapre string of expected reply to look for
		sResult << uppercase << command <<"=";

		circular_buffer<char> data = serialPort_->getDataRead();

		int pos = searchBufferR(data, sResult.str());
		int i = pos;

		char * dataCh = new char[data.size() - i];

		while(i < data.size()) {
			dataCh[i - pos] = data[i];
			i++;
		}

		sscanf( &dataCh[ command.length() ], "=%d:%d", result1, result2);

	}


    void RobotDrive::getRuntime(const std::string &command, int param, int *result) {
		int temp;
		getRuntime(command, param, result, &temp);

	}

	void RobotDrive::getRuntime(const std::string &command, int *result1, int *result2) {
		getRuntime(command, NO_PARAM, result1, result2);
	}


	bool RobotDrive::connectionTestOk() {
		static const int limit = 10;
		int count = 0;
		while(count < limit){
			serialPort_->write("?FID_\r");
			usleep(750000);

			circular_buffer<char> data = serialPort_->getDataRead();
			cout << "data.size() = " << data.size() << endl;
			if(data.size() >0){
				for(circular_buffer<char>::iterator it = data.begin(); it != data.end(); it++){
					cout << *it;
				}
				cout << endl;
			}

			int result = searchBufferR(data, "FID=");
			if (result >= 0){
				return true;
			}
		}

		return false;
	}

	void RobotDrive::searchForDevice() {
		cout << "Seeking for Roboteq driver\n";
		std::vector<std::string> comList;
		serialPort_->DetectComPorts(comList, 128);
		
		if(serialPort_->isActive()){
			serialPort_->close();
			delete serialPort_;
		}
		for(vector<string>::iterator port = comList.begin(); port != comList.end(); port++) {
			try {
				cout << "Checking "<< *port << endl;
				serialPort_ = new SerialPort(baud_, *port);
			}
			catch (...) {
				//cout << "continue;" << endl;
				continue;
			}
			if( connectionTestOk() ){
				cout << "Roboteq driver connected! Port: "<< *port <<endl;
				return;
			}
			else{
				cout << "Wrong port" << endl;
				serialPort_->close();
			}
		}
		delete serialPort_;
	}

	void RobotDrive::resume() {
		int speed = lastSpeed;
		int actualRpms = (int)(maxRpms *  abs(speed) / 1000);

		setConfig("MVEL",1,20);
		setConfig("MVEL",2,20);

		

		int countToGo1 = lastGoalL;
		int countToGo2 = lastGoalR;


		setRuntime("P",1, countToGo1);
		setRuntime("P",2, countToGo2);



		lastCommand = POSITION;
	}
}





/*************   commands ********************
**	MMOD - sets mode of operation; 1 - open-loop, 2 - speed, 3 - position
**	M - issues command to both channels
**  S - sets speed of a specific channel
**	EMOD - encoder mode
**	ELL - encoder low limit
*********************************************/
