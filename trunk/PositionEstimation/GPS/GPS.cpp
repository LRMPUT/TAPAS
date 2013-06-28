/*
 * GPS.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: jacek
 */
#include "GPS.h"

#include <cmath>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>

//EquatorialRadius
#define EqRd  6378.137
//PolarRadius
#define PlRd 6356.7523

GPS::GPS() {
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	Port = 0;
	Baud = 9600;
	FD = 0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
}

GPS::GPS(char *PortName, int BaudRate){
	PosX = 0.0;
	PosY = 0.0;
	PosLat = 0.0;
	PosLon = 0.0;
	Port = PortName;
	Baud = BaudRate;
	FD = 0;
	StartPosLat = 0.0;
	StartPosLon = 0.0;
	Radius = 0.0;
	openPort();
	start();
}

void GPS::initController(char *PortName, int BaudRate){
	Port = PortName;
	Baud = BaudRate;
	openPort();
	start();
}

GPS::~GPS() {
	nmea_parser_destroy(&Parser);
	close(FD);
}

//Otwiera port
int GPS::openPort(){
	struct termios toptions;
	bzero(&toptions, sizeof(toptions));

	FD = open(Port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (FD == -1)  {
		perror("init_serialport: Unable to open port \n");
		printf("Failed to open port \n");
		return -1;
	}
	else std::cout << "Port opened successfully \n";

	if (tcgetattr(FD, &toptions) < 0) {
		perror("init_serialport: Couldn't get term attributes \n");
		printf("Coulnd't get term attributes\n");
		return -1;
	}
	else printf("Term attributes got correctly\n");

	speed_t brate = Baud; // let you override switch below if needed
	switch(Baud) {
		case 4800:   brate=B4800;   break;
		case 9600:   brate=B9600;   break;
	#ifdef B14400
		case 14400:  brate=B14400;  break;
	#endif
		case 19200:  brate=B19200;  break;
	#ifdef B28800
		case 28800:  brate=B28800;  break;
	#endif
		case 38400:  brate=B38400;  break;
		case 57600:  brate=B57600;  break;
		case 115200: brate=B115200; break;
	}
	cfsetispeed(&toptions, brate);
	cfsetospeed(&toptions, brate);

	// 8N1
	toptions.c_cflag &= ~PARENB;
	toptions.c_cflag &= ~CSTOPB;
	toptions.c_cflag &= ~CSIZE;
	toptions.c_cflag |= CS8;
	// no flow control
	toptions.c_cflag &= ~CRTSCTS;

	toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    // toptions.c_lflag &= ~( ECHO | ECHOE | ISIG); // make raw
	//Canonical Input
	toptions.c_lflag |= ICANON;

	toptions.c_oflag |= OPOST; // make raw

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	toptions.c_cc[VMIN]  = 0;
	toptions.c_cc[VTIME] = 20;

	toptions.c_cc[VINTR]    = 0;     /* Ctrl-c */
	toptions.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	toptions.c_cc[VERASE]   = 0;     /* del */
	toptions.c_cc[VKILL]    = 0;     /* @ */
	toptions.c_cc[VEOF]     = 4;     /* Ctrl-d */
	toptions.c_cc[VTIME]    = 0;     /* inter-character timer unused */
	toptions.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
	toptions.c_cc[VSWTC]    = 0;     /* '\0' */
	toptions.c_cc[VSTART]   = 0;     /* Ctrl-q */
	toptions.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	toptions.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	toptions.c_cc[VEOL]     = 0;     /* '\0' */
	toptions.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	toptions.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	toptions.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	toptions.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	toptions.c_cc[VEOL2]    = 0;     /* '\0' */

	if( tcsetattr(FD, TCSANOW, &toptions) < 0) {
		perror("init_serialport: Couldn't set term attributes");
		return -1;
	}
	return 0;
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
    m_Thread = boost::thread(&GPS::monitorSerialPort, this);
}
void GPS::join()
{
    m_Thread.join();
}
void GPS::monitorSerialPort()
{
	boost::posix_time::milliseconds WorkTime2(50);
    nmea_zero_INFO(&Info);
    nmea_parser_init(&Parser);
	int N = 0;
	char * Ptr = Buffer;
	bool Success = false;
	int BuffLen = 0;
	int j=0;
	for(int i=0; i<7; i++){
		N = read(FD, Ptr, 255);
	}
	while(1){
		Success = false;
		Ptr = Buffer;
		N = 0;
		j=0;
		BuffLen = 0;
		ClearBuffer();
		nmea_parser_buff_clear(&Parser);
		do{
			N = read(FD, Ptr, 90);
			if (N > 0){
				Success = true;
				BuffLen += N;
				Ptr += N;
				j++;
			}
		}
		while ((N != -1) || j<5);
		boost::this_thread::sleep(WorkTime2);
		if (Success){
			Ptr = Buffer;
			nmea_parse(&Parser, Ptr, BuffLen, &Info);
			PosLat = nmea_ndeg2degree(Info.lat);
			PosLon = nmea_ndeg2degree(Info.lon);
		}
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
