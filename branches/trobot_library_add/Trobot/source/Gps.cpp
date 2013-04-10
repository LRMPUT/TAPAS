#include "../include/Gps.h"
#include <string>
#include <boost/circular_buffer.hpp>
#include "SerialPort.h"

using namespace std;

namespace trobot {


	Gps::Gps(unsigned int baud, const string& device)
	{
		baud_ = baud;
			try {
				serialPort_ = new SerialPort(baud, device);
			}
			catch (std::exception e) {
				searchForDevice();
			}
			if(!connectionTestOk())
				searchForDevice();
	}



	Gps::~Gps(void)
	{
	}



	GpsPosition Gps::getPosition() {
		if (serialPort_->newDataAvailable()) 
			parseData();
		return position_;
	}




	bool Gps::parseData() {
		// reading data from serial buffer
		buffer_ = serialPort_->getDataRead();
		index_ = serialPort_->max_read_length;
		
		bool startFound = false;
		while (!startFound) {
			bool endFound = lookForEnding();
			if(!endFound) 
				return false;
			startFound = lookForStart();
		}

		string temp_;
		index_ = start_; 
		double longt_;
		double latt_;
		char north_;
		char east_;
		int	fix_;

		temp_ = frameToString();
		/*   assigning frame data to variables. frame format is:
		**$--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx 
		**
		**	hhmmss.ss = UTC of position 
		**	 llll.ll = latitude of position
		**	 a = N or S
		**	 yyyyy.yy = Longitude of position
		**	 a = E or W 
		**	 x = GPS Quality indicator (0=no fix, 1=GPS fix, 2=Dif. GPS fix) 
		**	 xx = number of satellites in use 
		**	 x.x = horizontal dilution of precision 
		**	 x.x = Antenna altitude above mean-sea-level
		**	 M = units of antenna altitude, meters 
		**	 x.x = Geoidal separation
		**	 M = units of geoidal separation, meters 
		**	 x.x = Age of Differential GPS data (seconds) 
		**	 xxxx = Differential reference station ID
		************************************************************/
		sscanf(temp_.c_str(),"$GPGGA,%2d%2d%f,%2d%f,%c,%3d%f,%c,%d",
			&position_.time.hours,
			&position_.time.minutes,
			&position_.time.seconds,
			&position_.lattitude.degrees,
			&position_.lattitude.minutes,
			&north_,
			&position_.longtitude.degrees,
			&position_.longtitude.minutes,
			&east_,
			&fix_
			);
		if(north_ == 'N' || north_ =='n')
			position_.lattitude.north = true;
		else 
			position_.lattitude.north = false;

		if(east_ == 'E' || east_ == 'e')
			position_.longtitude.east = true;
		else 
			position_.longtitude.east = false;

		if(fix_ != 0)
			position_.fixQualityOk = true;
		else 
			position_.fixQualityOk = false;

	}


	bool Gps::lookForEnding() {
		if (index_ > buffer_.size() - 1)
			index_ = buffer_.size() - 2;
		for (index_; index_ >=0; index_--) {
			if((buffer_[ index_ ] == 0x0D) && (buffer_[ index_ + 1] == 0x0A)) {
				end_ = index_ + 1;
				return true;
			}
		}
		// return false if failed to find ending
		return false;
	}




	bool Gps::lookForStart() {
		if (index_ > buffer_.size() - 5)
			index_ = buffer_.size() - 6;
		for (index_; index_ >=0; index_--) {
			// looking for "$GPGGA" sentence marking beginning of the frame
			if( (buffer_[ index_ ] == '$') && (buffer_[ index_ + 1] == 'G') && (buffer_[ index_ + 2] == 'P') &&
				(buffer_[ index_ + 3] == 'G') && (buffer_[ index_ + 4] == 'G') && (buffer_[ index_ + 5] == 'A')) {
					start_ = index_;
					return true;
			}
		}
		return false;
	}




	string Gps::frameToString() {
		string result = "";
		while (index_ != end_) {
			result.append(&buffer_[ index_ ]);
			index_++;
		}
		return result;
	}

	bool Gps::connectionTestOk() {
		sleep(2);

		circular_buffer<char> data = serialPort_->getDataRead();

		if(searchBufferR(data, "$GPGGA") >= 0)
			return true;
		else
			return false;
	}

	void Gps::searchForDevice() {
		vector<string> comList;
		serialPort_->DetectComPorts(comList, 128);

		for(vector<string>::iterator port = comList.begin(); port != comList.end(); port++) {
			serialPort_->open(baud_, port);
			if( connectionTestOk() )
				return;
		}

	}

}
