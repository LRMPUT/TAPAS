/*
 * Recording.cpp
 *
 *  Created on: 01-07-2013
 *      Author: robots
 */

#include <algorithm>
#include "Recording.h"

using namespace std;
using namespace cv;

Recording::Recording(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug) :
		ui(iui),
		robot(irobot),
		debug(idebug)
{
	connect(&hokuyoTimer, SIGNAL(timeout()), this, SLOT(getDataHokuyo()));
	connect(&encodersTimer, SIGNAL(timeout()), this, SLOT(getDataEncoders()));
	connect(&gpsTimer, SIGNAL(timeout()), this, SLOT(getDataGps()));
	connect(&imuTimer, SIGNAL(timeout()), this, SLOT(getDataImu()));
	connect(ui->startRecButton, SIGNAL(clicked()), this, SLOT(startRec()));
	connect(ui->stopRecButon, SIGNAL(clicked()), this, SLOT(stopRec()));
	connect(ui->pauseResumeRecButton, SIGNAL(clicked()), this, SLOT(pauseResumeRec()));
}

Recording::~Recording(){

}


void Recording::getDataHokuyo(){

}

void Recording::getDataEncoders(){
	const Mat encoderData = debug->getEncoderData();
	encodersStream << time.elapsed() << " ";
	encodersStream << encoderData.at<int>(0) << " " << encoderData.at<int>(1) << endl;
}

void Recording::getDataGps(){
	//cout << "Recording::gatDataGps()" << endl;
	const Mat gpsData = debug->getGpsData();
	//cout << "Data dims = (" << gpsData.rows << ", " << gpsData.cols << ")" << endl;
	gpsStream<<time.elapsed()<<" ";
	gpsStream<<gpsData.at<float>(0)<<" "<<gpsData.at<float>(1)<<std::endl;
	//cout << "end Recording::getDataGps()" << endl;
}

void Recording::getDataImu(){
	Mat imuData = debug->getImuData();

	imuStream<<time.elapsed()<<" ";
	for(int val = 0; val < 4; val++){
		for(int i = 0; i < 3; i++){
			imuStream<<imuData.at<float>(i, val)<<" ";
		}
	}
	imuStream<<std::endl;
}

void Recording::getDataCamera(){

}


void Recording::startRec(){
	cout << "Recording::startRec()" << endl;
	ui->saRateGroupBox->setEnabled(false);
	/*file.open(ui->recPathLineEdit->text().toAscii().data(), ios_base::out);
	if(!file.is_open()){
		ui->recStatusLabel->setText("Could not open file");
		return;
	}*/

	// Moved it here, because sometimes checking if open can take seconds and other may
	// want to write sth to their streams
	time.start();

	if(ui->includeHokuyoCheckBox->isChecked() == true){
		if(!robot->isHokuyoOpen()){
			ui->recStatusLabel->setText("Hokuyo not active");
			stopRec();
			return;
		}
		hokuyoTimer.setInterval(max((int)(1000/ui->saRateHokuyoLineEdit->text().toFloat()), 1));
		hokuyoTimer.start();
	}
	if(ui->includeEncodersCheckBox->isChecked() == true){
		if(!robot->isEncodersOpen()){
			ui->recStatusLabel->setText("Encoders error");
			stopRec();
			return;
		}
		encodersStream.open("encoders.data");
		encodersTimer.setInterval(max((int)(1000/ui->saRateEncodersLineEdit->text().toFloat()), 1));
		encodersTimer.start();
	}
	if(ui->includeGpsCheckBox->isChecked() == true){
		if(!robot->isGpsOpen()){
			ui->recStatusLabel->setText("GPS error");
			stopRec();
			return;
		}
		gpsStream.open("gps.data");
		gpsTimer.setInterval(max((int)(1000/ui->saRateGpsLineEdit->text().toFloat()), 1));
		gpsTimer.start();
	}
	if(ui->includeImuCheckBox->isChecked() == true){
		if(!robot->isImuOpen()){
			ui->recStatusLabel->setText("IMU error");
			stopRec();
			return;
		}

		imuStream.open("imu.data");
		imuTimer.setInterval(max((int)(1000/ui->saRateImuLineEdit->text().toFloat()), 1));
		imuTimer.start();
	}
	cout << "end Recording::startRec()" << endl;
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
		encodersStream.close();
	}
	if(ui->includeGpsCheckBox->isChecked() == true){
		gpsTimer.stop();
		gpsStream.close();
	}
	if(ui->includeImuCheckBox->isChecked() == true){
		imuTimer.stop();
		imuStream.close();
	}
	ui->saRateGroupBox->setEnabled(true);
	cout << "end Recording::stopRec()" << endl;
}
