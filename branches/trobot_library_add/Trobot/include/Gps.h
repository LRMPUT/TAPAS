#ifndef GPS_H
#define GPS_H

#include <string>
#include <boost/circular_buffer.hpp>
#include "../include/SerialPort.h"


namespace trobot {
	struct Longtitude {
		int		degrees;
		float	minutes;
		bool	east;
	};
	struct Lattitude {
		int		degrees;
		float	minutes;
		bool	north;
	};
	struct GpsTime {
		int		hours;
		int		minutes;
		float	seconds;
	};
	struct	GpsPosition {
		Longtitude	longtitude;
		Lattitude	lattitude;
		GpsTime		time;
		bool		fixQualityOk;
	};

	class Gps
	{
	public:
		Gps(unsigned int baud, const std::string& device);
	public:
		~Gps(void);

		GpsPosition		getPosition(void);

	private:
		bool	parseData(void);  ///< parse NIMEA frame data, looking for $GPGGA command, returns true if succeeded
		bool	lookForStart(void); ///< looks for beginning of $GPGGA frame, returns true if succeeded
		bool	lookForEnding(void); 
		std::string	frameToString(void); ///< writes chars between commas into a string

		void	searchForDevice();	///< searches for a device and connects to it
		bool	connectionTestOk(); ///< tests if the connection to the controller was properly established
		
		SerialPort	*			serialPort_;
		unsigned int			baud_;
		GpsPosition				position_;
		boost::circular_buffer<char>	buffer_;
		unsigned int			index_;
		int						end_;
		int						start_;

		

	};

}

#endif //GPS_H
