/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology

    This library is free software; you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation; either version 2.1 of the License, or (at your option)
    any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
    details.

    You should have received a copy of the GNU Lesser General Public License along
    with this library; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

*/

#include <algorithm>
#include "Recording.h"

using namespace std;
using namespace cv;

Recording::Recording(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug) :
		ui(iui), robot(irobot), debug(idebug) {
	connect(&hokuyoTimer, SIGNAL(timeout()), this, SLOT(getDataHokuyo()));
	connect(&encodersTimer, SIGNAL(timeout()), this, SLOT(getDataEncoders()));
	connect(&gpsTimer, SIGNAL(timeout()), this, SLOT(getDataGps()));
	connect(&cameraTimer, SIGNAL(timeout()), this, SLOT(getDataCamera()));
	connect(&imuTimer, SIGNAL(timeout()), this, SLOT(getDataImu()));
	connect(&estimatedPosTimer, SIGNAL(timeout()), this,
			SLOT(getDataEstimatedPos()));
	connect(ui->startRecButton, SIGNAL(clicked()), this, SLOT(startRec()));
	connect(ui->stopRecButon, SIGNAL(clicked()), this, SLOT(stopRec()));
	connect(ui->pauseResumeRecButton, SIGNAL(clicked()), this,
			SLOT(pauseResumeRec()));
}

Recording::~Recording() {

}

void Recording::getDataHokuyo() {
	const Mat hokuyoData = debug->getHokuyoData();
	hokuyoStream << time.elapsed() << endl << "d ";
	for (int c = 0; c < hokuyoData.cols; c++) {
		hokuyoStream << hokuyoData.at<int>(2, c) << " ";
	}
	hokuyoStream << endl;
	hokuyoStream << "i ";
	for (int c = 0; c < hokuyoData.cols; c++) {
		hokuyoStream << hokuyoData.at<int>(3, c) << " ";
	}
	hokuyoStream << endl;
}

void Recording::getDataEncoders() {
	const Mat encoderData = debug->getEncoderData();
	encodersStream << time.elapsed() << " ";
	encodersStream << encoderData.at<int>(0) << " " << encoderData.at<int>(1)
			<< endl;
}

void Recording::getDataGps() {
	//cout << "Recording::gatDataGps()" << endl;
	const Mat gpsData = debug->getGpsData();
	//cout << "Data dims = (" << gpsData.rows << ", " << gpsData.cols << ")" << endl;
	gpsStream << time.elapsed() << " ";
	gpsStream << gpsData.at<float>(0) << " " << gpsData.at<float>(1);
	//cout << gpsData.at<float>(2) << " " << gpsData.at<float>(3) << std::endl;
	gpsStream << " " << gpsData.at<float>(2) << " " << gpsData.at<float>(3)
			<< std::endl;
	//cout << "end Recording::getDataGps()" << endl;
}

void Recording::getDataImu() {
	Mat imuData = debug->getImuData();

	imuStream << time.elapsed() << " ";
	for (int i = 0; i < 3; i++) {
		for (int val = 0; val < 4; val++) {
			imuStream << imuData.at<float>(i, val) << " ";
		}
	}
	imuStream << std::endl;
}

void Recording::getDataCamera() {
	//cout << "Getting camera data" << endl;
	static int index = 0;
	static const QString cameraPrefix("rec/camera");
	static const QString cameraExt(".jpg");
	vector<int> jpegParams;
	jpegParams.push_back(CV_IMWRITE_JPEG_QUALITY);
	jpegParams.push_back(100);
	QString fileName = cameraPrefix
			+ QString("%1").arg(index, 3, 10, QChar('0')) + cameraExt;
	cameraStream << time.elapsed() << " " << fileName.toAscii().data() << endl;
	vector<Mat> cameraData = debug->getCameraData();
	imwrite(fileName.toAscii().data(), cameraData.front(), jpegParams);
	index++;
}

void Recording::getDataEstimatedPos() {
	Mat estimatedPos = debug->getEstimatedPosition();
	estimatedPosStream << time.elapsed() << " " << estimatedPos.at<double>(0)
			<< " " << estimatedPos.at<double>(1) << " "
			<< estimatedPos.at<double>(2)<<" " << estimatedPos.at<double>(3);
}

void Recording::startRec() {
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

	if (ui->includeHokuyoCheckBox->isChecked() == true) {
		if (!robot->isHokuyoOpen()) {
			ui->recStatusLabel->setText("Hokuyo not active");
			stopRec();
			return;
		}
		hokuyoStream.open("rec/hokuyo.data");
		hokuyoTimer.setInterval(
				max((int) (1000 / ui->saRateHokuyoLineEdit->text().toFloat()),
						1));
		hokuyoTimer.start();
	}
	if (ui->includeEncodersCheckBox->isChecked() == true) {
		if (!robot->isEncodersOpen()) {
			ui->recStatusLabel->setText("Encoders error");
			stopRec();
			return;
		}
		encodersStream.open("rec/encoders.data");
		encodersTimer.setInterval(
				max((int) (1000 / ui->saRateEncodersLineEdit->text().toFloat()),
						1));
		encodersTimer.start();
	}
	if (ui->includeGpsCheckBox->isChecked() == true) {
		if (!robot->isGpsOpen()) {
			ui->recStatusLabel->setText("GPS error");
			stopRec();
			return;
		}
		gpsStream.open("rec/gps.data");
		gpsStream.precision(15);
		gpsTimer.setInterval(
				max((int) (1000 / ui->saRateGpsLineEdit->text().toFloat()), 1));
		gpsTimer.start();
	}
	if (ui->includeCamerasCheckBox->isChecked() == true) {
		if (!robot->isCameraOpen()) {
			ui->recStatusLabel->setText("Camera error");
			stopRec();
			return;
		}
		cameraStream.open("rec/camera.data");
		cameraTimer.setInterval(
				max((int) (1000 / ui->saRateCamerasLineEdit->text().toFloat()),
						1));
		//cout << "Starting camera timer with interval "
		//		<< max((int)(1000/ui->saRateCamerasLineEdit->text().toFloat()), 1) << endl;
		cameraTimer.start();
	}
	if (ui->includeImuCheckBox->isChecked() == true) {
		if (!robot->isImuOpen()) {
			ui->recStatusLabel->setText("IMU error");
			stopRec();
			return;
		}

		imuStream.open("rec/imu.data");
		imuStream.precision(15);
		imuTimer.setInterval(
				max((int) (1000 / ui->saRateImuLineEdit->text().toFloat()), 1));
		imuTimer.start();
	}
	if (ui->includeImuCheckBox->isChecked() == true) {
		estimatedPosStream.open("rec/estimatedPos.data");
		estimatedPosStream.precision(15);
		estimatedPosTimer.setInterval(
				max((int) (1000 / ui->saRateImuLineEdit->text().toFloat()), 1));
		estimatedPosTimer.start();
	}
	ui->recStatusLabel->setText("Started");
	cout << "end Recording::startRec()" << endl;
}

void Recording::pauseResumeRec() {
	if (ui->pauseResumeRecButton->text().compare("Pause") == 0) {
		ui->pauseResumeRecButton->setText("Resume");
		if (ui->includeHokuyoCheckBox->isChecked() == true) {
			hokuyoTimer.stop();
		}
		if (ui->includeEncodersCheckBox->isChecked() == true) {
			encodersTimer.stop();
		}
		if (ui->includeGpsCheckBox->isChecked() == true) {
			gpsTimer.stop();
		}
		if (ui->includeCamerasCheckBox->isChecked() == true) {
			cameraTimer.stop();
		}
		if (ui->includeImuCheckBox->isChecked() == true) {
			imuTimer.stop();
		}
		if (ui->includeImuCheckBox->isChecked() == true) {
			estimatedPosTimer.stop();
		}
		ui->recStatusLabel->setText("Paused");
	} else {
		ui->pauseResumeRecButton->setText("Pause");
		if (ui->includeHokuyoCheckBox->isChecked() == true) {
			hokuyoTimer.start();
		}
		if (ui->includeEncodersCheckBox->isChecked() == true) {
			encodersTimer.start();
		}
		if (ui->includeGpsCheckBox->isChecked() == true) {
			gpsTimer.start();
		}
		if (ui->includeCamerasCheckBox->isChecked() == true) {
			cameraTimer.start();
		}
		if (ui->includeImuCheckBox->isChecked() == true) {
			imuTimer.start();
		}
		if (ui->includeImuCheckBox->isChecked() == true) {
			estimatedPosTimer.start();
		}
		ui->recStatusLabel->setText("Resumed");
	}
}

void Recording::stopRec() {
	if (ui->includeHokuyoCheckBox->isChecked() == true) {
		hokuyoTimer.stop();
		hokuyoStream.close();
	}
	if (ui->includeEncodersCheckBox->isChecked() == true) {
		encodersTimer.stop();
		encodersStream.close();
	}
	if (ui->includeGpsCheckBox->isChecked() == true) {
		gpsTimer.stop();
		gpsStream.close();
	}
	if (ui->includeCamerasCheckBox->isChecked() == true) {
		cameraTimer.stop();
		cameraStream.close();
	}
	if (ui->includeImuCheckBox->isChecked() == true) {
		imuTimer.stop();
		imuStream.close();
	}
	if (ui->includeImuCheckBox->isChecked() == true) {
		estimatedPosTimer.stop();
		estimatedPosStream.close();
	}
	ui->saRateGroupBox->setEnabled(true);
	ui->recStatusLabel->setText("Stopped");
	cout << "end Recording::stopRec()" << endl;
}
