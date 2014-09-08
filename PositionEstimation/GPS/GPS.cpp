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
//C library
#include <cmath>
#include <cstring>
//RobotsIntellect
#include "GPS.h"

using namespace std;
using namespace boost;

//EquatorialRadius [mm]
#define EqRd  6378137000.0
//PolarRadius [mm]
#define PlRd 6356752300.0

GPS::GPS() {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
	newMeasurement = false;
}

GPS::GPS(Robot* irobot) : robot(irobot) {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
	newMeasurement = false;
}

GPS::GPS(const char *PortName, int BaudRate) {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
	newMeasurement = false;
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
//	printf("GPS serial port active state : %d\n", SerialPort.isActive());
	return SerialPort.isActive();
}

std::chrono::high_resolution_clock::time_point GPS::getTimestamp() {
	return timestamp;
}


double GPS::getPosX(){
	newMeasurement = false;
	PosX = (nmea_ndeg2radian(Info.lat - StartPosLat))*Radius;
	//cout << "Angular difference X = " << Info.lon << " - " << StartPosLon << " = " <<
	//		 nmea_ndeg2radian(Info.lon - StartPosLon) << endl;
	return PosX;
}

double GPS::getPosX(double latitude)
{
	return (nmea_ndeg2radian(latitude - StartPosLat))*Radius;
}

double GPS::getPosLatitude(double Y)
{
	return (nmea_radian2ndeg(Y/Radius) + StartPosLat);
}


double GPS::getPosY(){
	newMeasurement = false;
	PosY = (nmea_ndeg2radian(Info.lon - StartPosLon))*Radius;
	return PosY;
}

double GPS::getPosY(double longitude)
{
	return (nmea_ndeg2radian(longitude - StartPosLon))*Radius;
}

double GPS::getPosLongitude(double Y)
{
	return (nmea_radian2ndeg(Y/Radius) + StartPosLon);
}

double GPS::getLat(){
	return Info.lat;
}


double GPS::getLon(){
	return Info.lon;
}

double GPS::getHorPrec(){
	return Info.HDOP;
}

int GPS::getFixStatus(){
	return Info.fix;
}

int GPS::getSatelitesUsed(){
	return Info.satinfo.inuse + 100*Info.satinfo.inview;
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
	boost::posix_time::milliseconds SleepTime(200);
    nmea_zero_INFO(&Info);
    nmea_parser_init(&Parser);
	nmea_parser_buff_clear(&Parser);
	while(1){
		circular_buffer<char> data = SerialPort.getDataRead();
		//cout << "Read " << data.size() << endl;

		int cnt = 0;
		circular_buffer<char>::array_range rg = data.array_one();
		memcpy(Buffer, rg.first, rg.second);
		cnt += rg.second;

		rg = data.array_two();
		memcpy(Buffer + cnt, rg.first, rg.second);
		cnt += rg.second;

		//int cnt = data.size();
		/*for(int i = 0; i < data.size(); i++){
			//Buffer[i] = data[i];
			cout << Buffer[i];
		}
		cout << endl;*/
		if(cnt > 0){
			int packetsRead = nmea_parse(&Parser, Buffer, cnt, &Info);
			//cout << "GPS parsed " << packetsRead << endl;
			if(packetsRead > 0){
				timestamp = std::chrono::high_resolution_clock::now();
				PosLat = nmea_ndeg2degree(Info.lat);
				PosLon = nmea_ndeg2degree(Info.lon);
				newMeasurement = true;
			}
		}
		if(threadEnd == true){
			return;
		}
		//cout << "GPS parse sleeping" << endl;
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
		printf("Our GPS issue\n\n\n\n\n");
		return -1;
	}
	double StartPosLatRad = nmea_ndeg2radian(StartPosLat);
	Radius = sqrt( ((pow( pow(EqRd,2.0)*cos(StartPosLatRad),2.0)) + (pow( pow( PlRd ,2.0)*sin(StartPosLatRad),2.0)))
			/ ((pow( EqRd*cos(StartPosLatRad),2.0)) + (pow( PlRd*sin(StartPosLatRad),2.0))));
	cout << "Radius = " << Radius << endl;
	return 0;
}

bool GPS::getNewMeasurement()
{
	return newMeasurement;
}
