/*
 * Recording.cpp
 *
 *  Created on: 01-07-2013
 *      Author: robots
 */
#include "Recording.h"
#include <algorithm>

using namespace std;

Recording::Recording(Ui::TrobotQtClass* iui) : ui(iui){
	connect(&hokuyoTimer, SIGNAL(timeout()), this, SLOT(getDataHokuyo()));
	connect(&encodersTimer, SIGNAL(timeout()), this, SLOT(getDataEncoders()));
	connect(&gpsTimer, SIGNAL(timeout()), this, SLOT(getDataGps()));
	connect(&imuTimer, SIGNAL(timeout()), this, SLOT(getDataImu()));
}

Recording::~Recording(){

}


void Recording::getDataHokuyo(){

}

void Recording::getDataEncoders(){

}

void Recording::getDataGps(){

}

void Recording::getDataImu(){

}

void Recording::getDataCameras(){

}


void Recording::startRec(trobot::Imu* imu, trobot::RobotDrive* drive, GPS* gps, urg_t* hokuyo){
	ui->saRateGroupBox->setEnabled(false);
	file.open(ui->recPathLineEdit->text().toAscii().data(), ios_base::out);
	if(!file.is_open()){
		ui->recStatusLabel->setText("Could not open file");
		return;
	}
	if(ui->includeHokuyoCheckBox->isChecked() == true){
		if(!hokuyo->is_active){
			ui->recStatusLabel->setText("Hokuyo not active");
			return;
		}
		hokuyoTimer.setInterval(max(1/ui->saRateHokuyoLineEdit->text().toFloat(), 1));
		hokuyoTimer.start();
	}
	if(ui->includeEncodersCheckBox->isChecked() == true){
		if(drive == NULL){	//TODO there should be functions to check if it's open
			ui->recStatusLabel->setText("Robot's drive error");
			return;
		}
		encodersTimer.setInterval(max(1/ui->saRateEncodersLineEdit->text().toFloat(), 1));
		encodersTimer.start();
	}
	if(ui->includeGpsCheckBox->isChecked() == true){
		if(gps == NULL){	//TODO there should be functions to check if it's open
			ui->recStatusLabel->setText("GPS error");
			return;
		}
		gpsTimer.setInterval(max(1/ui->saRateGpsLineEdit->text().toFloat(), 1));
		gpsTimer.start();
	}
	if(ui->includeImuCheckBox->isChecked() == true){
		if(imu == NULL){	//TODO there should be functions to check if it's open
			ui->recStatusLabel->setText("IMU error");
			return;
		}
		imuTimer.setInterval(max(1/ui->saRateImuLineEdit->text().toFloat(), 1));
		imuTimer.start();
	}
	time.start();
}

void Recording::pauseResumeRec(){
	if(ui->pauseResumeRecButton->text().compare("Pause") == 0){
		ui->pauseResumeRecButton->setText("Resume");
		if(ui->includeHokuyoCheckBox->isChecked() == true){
			hokuyoTimer.stop();
		}
		if(ui->includeEncodersCheckBox->isChecked() == true){
			encodersTimer.stop();
		}
		if(ui->includeGpsCheckBox->isChecked() == true){
			gpsTimer.stop();
		}
		if(ui->includeImuCheckBox->isChecked() == true){
			imuTimer.stop();
		}
	}
	else{
		ui->pauseResumeRecButton->setText("Pause");
		if(ui->includeHokuyoCheckBox->isChecked() == true){
			hokuyoTimer.start();
		}
		if(ui->includeEncodersCheckBox->isChecked() == true){
			encodersTimer.start();
		}
		if(ui->includeGpsCheckBox->isChecked() == true){
			gpsTimer.start();
		}
		if(ui->includeImuCheckBox->isChecked() == true){
			imuTimer.start();
		}
	}
}

void Recording::stopRec(){
	if(ui->includeHokuyoCheckBox->isChecked() == true){
		hokuyoTimer.stop();
	}
	if(ui->includeEncodersCheckBox->isChecked() == true){
		encodersTimer.stop();
	}
	if(ui->includeGpsCheckBox->isChecked() == true){
		gpsTimer.stop();
	}
	if(ui->includeImuCheckBox->isChecked() == true){
		imuTimer.stop();
	}
	file.close();
}
