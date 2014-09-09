/*
 *  @file GPS.h
 *  @author Jacek Jankowski
 *  @brief Handling of GPS LEA 6H for obtaining position
 */

#ifndef GPS_H_
#define GPS_H_


#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <chrono>
#include <iostream>

#include "../../Trobot/include/SerialPort.h"

#include "nmea/nmea.h"

#define BUFFER_SIZE 2048

class Debug;
class Robot;

/**
 * GPS Class
 */
class GPS {
	friend class Debug;
	Robot *robot;
public:
	/**
	 * Initiates all variables. You need to use @see initController() for starting the module
	 */
	GPS();
	/**
	 * Initiates all variables. You need to use @see initController() for starting the module
	 */
	GPS(Robot* robot);
	/**
	 * Initiates all variables and starts the module.
	 * @param[in] PortName Pointer to array containing name of port, to which the GPS module is attached
	 * @param[in] BaudRate Baud rate for serial communication with module, default 9600.
	 */
	GPS(const char *PortName, int BaudRate = 9600);
	/**
	 * Start the module. Use with default constructor.
	 * @param[in] PortName Pointer to array containing name of port, to which the GPS module is attached
	 * @param[in] BaudRate Baud rate for serial communication with module, default 9600.
	 */
	void initController(const char *PortName, int BaudRate);
	/**
	 * Stops the module
	 */
	void deinitController();
	/**
	 * Default virtual destructor.
	 */
	virtual ~GPS();

	/*
	 * @return Timestamp of the measurement.
	 */
	std::chrono::high_resolution_clock::time_point getTimestamp();
	/*
	* @return Bool indicating if there has been a new measurement from the last read.
	*/
	bool getNewMeasurement();

	/*
	 * Conversion between Nmea and decimal format of latitude and longitude
	 */
	static double nmea2Decimal(double value);
	static double decimal2Nmea(double value);
	/*
	 * @return Distance from Zero Point measured in meters, X (latitude - poludnikowo - szerokosc geograficzna).
	 */
	double getPosX();
	double getPosX(double latitude);
	double getPosLatitude(double X);
	/*
	 * @return Distance from Zero Point measured in meters, Y (longitude - rownoleznikowo - dlugosc geograficzna).
	 */
	double getPosY();
	double getPosY(double longitude);
	double getPosLongitude(double Y);
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

	int getFixStatus();

	int getSatelitesUsed();
	/*
	 * Used to set the Zero Point for measuring relative distance.
	 * By default the X and Y axis are oriented as latitude and longitude respectively.
	 * @warning Use only when fix available.
	 */
	void setZeroXY(double Latitude, double Longitude);
	bool getIsSetZero();
	/**
	 * Checks if port is opened
	 */
	bool isOpen();

private:
	double PosLat, PosLon;
	double PosX, PosY;
	double StartPosLat, StartPosLon;
	trobot::SerialPort SerialPort;
	double Radius;
	char Buffer[BUFFER_SIZE];
	void ClearBuffer();
	volatile bool threadEnd;
	bool isSetZero;

	void start();
	void join();
	int openPort(const char* port, int BaudRate);
	void closePort();
	void monitorSerialPort();
	int calculateRadius();

	std::chrono::high_resolution_clock::time_point timestamp;
	bool newMeasurement;

	boost::thread m_Thread;
	nmeaINFO Info;
	nmeaPARSER Parser;
};

#endif /* GPS_H_ */
