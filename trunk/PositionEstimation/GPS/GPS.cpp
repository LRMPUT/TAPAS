/*
 * GPS.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: jacek
 */

//STL
#include <iostream>
//Boost
#include <boost/circular_buffer.hpp>

#include <cmath>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
//RobotsIntellect
#include "GPS.h"

using namespace std;
using namespace boost;

//EquatorialRadius
#define EqRd  6378.137
//PolarRadius
#define PlRd 6356.7523

GPS::GPS() {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
}

GPS::GPS(const char *PortName, int BaudRate) {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
	initController(PortName, BaudRate);
}

void GPS::initController(const char *PortName, int BaudRate){
	openPort(PortName, BaudRate);
	start();
}

void GPS::deinitController(){
	if(SerialPort.isActive()){
		cout << "Joining thread" << endl;
		join();
		cout << "Destroying parser" << endl;
		nmea_parser_destroy(&Parser);
		cout << "Closing port" << endl;
		closePort();
	}
}

GPS::~GPS() {
	deinitController();
}

//Otwiera port
int GPS::openPort(const char* port, int BaudRate){
	SerialPort.open(BaudRate, port);

	return 0;
}

void GPS::closePort()
{
	SerialPort.close();
}

bool GPS::isOpen()
{
	return SerialPort.isActive();
}

double GPS::getPosX(){
	PosX = (nmea_ndeg2degree(Info.lon) - StartPosLon)*Radius;
	return PosX;
}

double GPS::getPosY(){
	PosY = (nmea_ndeg2degree(Info.lat) - StartPosLat)*Radius;
	return PosY;
}

double GPS::getLat(){
	return PosLat;
}

double GPS::getLon(){
	return PosLon;
}

double GPS::getHorPrec(){
	return Info.HDOP;
}

void GPS::setZeroXY(double Latitude, double Longitude){
	StartPosLat = Latitude;
	StartPosLon = Longitude;
	calculateRadius();
}

void GPS::start()
{
	threadEnd = false;
    m_Thread = boost::thread(&GPS::monitorSerialPort, this);
}

void GPS::join()
{
	threadEnd = true;
    m_Thread.join();
}

void GPS::monitorSerialPort()
{
	cout << "Starting monitoring serial port" << endl;
	boost::posix_time::milliseconds SleepTime(100);
    nmea_zero_INFO(&Info);
    nmea_parser_init(&Parser);
	nmea_parser_buff_clear(&Parser);
	while(1){
		circular_buffer<char> data = SerialPort.getDataRead();
		cout << "Read " << data.size() << endl;

		/*int cnt = 0;
		circular_buffer<char>::array_range rg = data.array_one();
		memcpy(Buffer, rg.first, cnt);
		cnt += rg.second;

		rg = data.array_two();
		memcpy(Buffer + cnt, rg.first, rg.second);
		cnt += rg.second;*/

		int cnt = data.size();
		for(int i = 0; i < data.size(); i++){
			Buffer[i] = data[i];
			cout << Buffer[i];
		}
		cout << endl;
		if(cnt > 0){
			int packetsRead = nmea_parse(&Parser, Buffer, cnt, &Info);
			cout << "Parsed " << packetsRead << endl;
			if(packetsRead > 0){
				PosLat = nmea_ndeg2degree(Info.lat);
				PosLon = nmea_ndeg2degree(Info.lon);
			}
		}
		if(threadEnd == true){
			return;
		}
		cout << "GPS parse sleeping" << endl;
		boost::this_thread::sleep(SleepTime);
	}
}
void GPS::ClearBuffer(){
	for(int i=0; i<2048; i++){
		Buffer[i]=0;
	}
}

int GPS::calculateRadius(){
	if(StartPosLat==0.0 || StartPosLon==0.0){
		return -1;
	}
	Radius = sqrt( ((pow( pow(EqRd,2.0)*cos(StartPosLat),2.0)) + (pow( pow( PlRd ,2.0)*sin(StartPosLat),2.0)))
			/ ((pow( EqRd*cos(StartPosLat),2.0)) + (pow( PlRd*sin(StartPosLat),2.0))));
	return 0;
}
