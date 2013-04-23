/*
 *  @file GPS.h
 *  @author Jacek Jankowski
 *  @brief Handling of GPS LEA 6H for obtaining position
 */

#ifndef GPS_H_
#define GPS_H_


#include <boost/thread.hpp>
#include "nmea/nmea.h"

/**
 * GPS Class
 */
class GPS {
public:
	/**
	 * Initiates all variables. You need to use @see initController() for starting the module
	 */
	GPS();
	/**
	 * Initiates all variables and starts the module.
	 * @param[in] PortName Pointer to array containing name of port, to which the GPS module is attached
	 * @param[in] BaudRate Baud rate for serial communication with module, default 9600.
	 */
	GPS(char *PortName, int BaudRate);
	/**
	 * Start the module. Use with default constructor.
	 * @param[in] PortName Pointer to array containing name of port, to which the GPS module is attached
	 * @param[in] BaudRate Baud rate for serial communication with module, default 9600.
	 */
	void initController(char *PortName, int BaudRate);
	/**
	 * Default virtual destructor.
	 */
	virtual ~GPS();

	/*
	 * @return Distance from Zero Point measured in meters, X (longitude - rownoleznikowo).
	 */
	double getPosX();
	/*
	 * @return Distance from Zero Point measured in meters, Y (latitude - poludnikowo).
	 */
	double getPosY();
	/*
	 * @return Current Latitude in degrees. GPS format is translated from +-[DDD][MM].[S].
	 */
	double getLat();
	/*
	 * @return Current Longitude in degrees. GPS format is translated from +-[DDD][MM].[S].
	 */
	double getLon();
	/*
	 * @return Horizontal Dilusion of Precision. See the attached .txt file HDOP_info.txt
	 *
	 */
	double getHorPrec();
	/*
	 * Used to set the Zero Point for measuring relative distance.
	 * By default the X and Y axis are oriented as latitude and longitude respectively.
	 * @warning Use only when fix available.
	 */
	void setZeroXY(double Latitude, double Longitude);

private:
	double PosLat, PosLon;
	double PosX, PosY;
	double StartPosLat, StartPosLon;
	char* Port;
	int Baud;
	int FD;
	double Radius;
	char Buffer[2048];
	void ClearBuffer();

	void start();
	void join();
	int openPort();
	void monitorSerialPort();
	int calculateRadius();

	boost::thread m_Thread;
	nmeaINFO Info;
	nmeaPARSER Parser;
};

#endif /* GPS_H_ */
