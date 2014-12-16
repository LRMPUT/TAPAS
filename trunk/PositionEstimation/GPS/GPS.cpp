/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

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
	RadiusLon = 0.0;
	newMeasurement = false;
	isSetZero = false;
	dataValid = false;
}

GPS::GPS(Robot* irobot) : robot(irobot) {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	RadiusLon = 0.0;
	RadiusLat = 0.0;
	newMeasurement = false;
	isSetZero = false;
	dataValid = false;
}

GPS::GPS(const char *PortName, int BaudRate) {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	RadiusLon = 0.0;
	RadiusLat = 0.0;
	newMeasurement = false;
	isSetZero = false;
	dataValid = false;
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

bool GPS::isDataValid(){
	return dataValid;
}

std::chrono::high_resolution_clock::time_point GPS::getTimestamp() {
	return timestamp;
}


double GPS::nmea2Decimal(double value) {
	return nmea_ndeg2degree(value) * 100.0;
}

double GPS::decimal2Nmea(double value) {
	return nmea_degree2ndeg(value/100);
}

double GPS::decimal2radian(double value) {
	return  nmea_degree2radian(value);
}

double GPS::getPosX(){
	newMeasurement = false;

	std::unique_lock<std::mutex> gpsLck(gpsDataMtx);
	PosX = (nmea_ndeg2radian(Info.lat) - nmea_ndeg2radian(StartPosLat))*RadiusLat;
	gpsLck.unlock();

	return PosX;
}

double GPS::getPosX(double startLatitude)
{
//    cout<< "GPS: " << StartPosLat << " " << StartPosLon << " vs 1st: " << latitude <<endl;
//	cout<< "Conversion -- RadiusLat : "<< RadiusLat<<std::endl;
//	printf("TEST : %.8f - %.8f =  %.8f\n", nmea_ndeg2radian(latitude), nmea_ndeg2radian(StartPosLat), nmea_ndeg2radian(latitude) - nmea_ndeg2radian(StartPosLat));

	return (nmea_ndeg2radian(startLatitude) - nmea_ndeg2radian(StartPosLat))*RadiusLat;
}

double GPS::getPosLatitude(double X)
{
	return nmea_radian2ndeg(X/RadiusLat + nmea_ndeg2radian(StartPosLat));
}


double GPS::getPosY(){
	newMeasurement = false;
	std::unique_lock<std::mutex> gpsLck(gpsDataMtx);
	PosY = (nmea_ndeg2radian(Info.lon) - nmea_ndeg2radian(StartPosLon))*RadiusLon;
	gpsLck.unlock();
	return PosY;
}

double GPS::getPosY(double startLongitude)
{
//	cout<< "GPS: " << StartPosLat << " " << StartPosLon << " vs 2nd:" << longitude <<endl;
//	cout<< "Conversion -- RadiusLon : "<< RadiusLon<<std::endl;
//
//	printf("TEST : %.8f - %.8f =  %.8f\n", nmea_ndeg2radian(longitude), nmea_ndeg2radian(StartPosLon), nmea_ndeg2radian(longitude) - nmea_ndeg2radian(StartPosLon));
	return (nmea_ndeg2radian(startLongitude) - nmea_ndeg2radian(StartPosLon))*RadiusLon;
}

double GPS::getPosLongitude(double Y)
{
	return nmea_radian2ndeg(Y/RadiusLon +nmea_ndeg2radian(StartPosLon));
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
	isSetZero = true;
}

bool GPS::getIsSetZero()
{
	return isSetZero;
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
	try{
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
					dataValid = true;
				}
			}
			if(threadEnd == true){
				return;
			}
			//cout << "GPS parse sleeping" << endl;
			boost::this_thread::sleep(SleepTime);
		}
	}
	catch(char const* error){
		cout << "Char exception in Gps: " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in Gps: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in Gps" << endl;
		exit(1);
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
	RadiusLat = sqrt( ((pow( pow(EqRd,2.0)*cos(StartPosLatRad),2.0)) + (pow( pow( PlRd ,2.0)*sin(StartPosLatRad),2.0)))
			/ ((pow( EqRd*cos(StartPosLatRad),2.0)) + (pow( PlRd*sin(StartPosLatRad),2.0))));
	RadiusLon = RadiusLat * cos(StartPosLatRad);
	cout << "Radius = " << RadiusLon << endl;
	cout << "RadiusLat = " << RadiusLat << endl;
	return 0;
}

bool GPS::getNewMeasurement()
{
	return newMeasurement;
}

void GPS::fakeGPSStart(double lat, double lon)
{
	StartPosLat = lat;
	StartPosLon = lon;
	calculateRadius();
}
