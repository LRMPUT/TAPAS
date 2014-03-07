/*
 * QtHokuyo.cpp
 *
 *  Created on: 12-11-2013
 *      Author: jachu
 */

//STL
#include <algorithm>
//Qt
#include <QtGui/QPainter>
#include <QtGui/QColor>
#include <QtGui/QPixmap>
#include <QtCore/QVector>
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot Qt
#include "QtHokuyo.h"

using namespace std;
using namespace cv;

QtHokuyo::QtHokuyo(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug) :
	ui(iui),
	robot(irobot),
	debug(idebug)
{
	QObject::connect(ui->hokuyoConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->hokuyoDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));

	refreshTimer.setInterval(100);

	map = QPixmap(ui->calibLaserLabel->width(), ui->calibLaserLabel->height());
}

QtHokuyo::~QtHokuyo(){
}

cv::Mat QtHokuyo::getData(){
	return debug->getHokuyoData();
}

void QtHokuyo::refresh(){
	Mat data = getData();

	map.fill(Qt::white);
	QPainter painter(&map);
	painter.setPen(Qt::blue);

	float scale;
	const int range = 3000;
	scale = (float)min(ui->calibLaserLabel->width(), ui->calibLaserLabel->height()) / (2 * range);

	int origX = ui->calibLaserLabel->width()/2;
	int origY = ui->calibLaserLabel->height()/2;

	QVector<QPointF> lines;
	for(int i = 0; i < data.cols; i++){
		//cout << "Point " << i << " = (" << data.at<int>(0, i) << ", " << data.at<int>(1, i) << ")" << endl;
		lines.append(QPointF(origX - data.at<int>(1, i)*scale, origY - data.at<int>(0, i)*scale));
	}
	painter.drawPolyline(lines);

	painter.end();
	ui->calibLaserLabel->setPixmap(map);
	ui->calibLaserLabel->update();
}

void QtHokuyo::connect(){
	if(ui->hokuyoPortCombo->count() != 0){
		robot->openHokuyo(ui->gpsPortCombo->currentText().toAscii().data());
		refreshTimer.start();
	}
}

void QtHokuyo::disconnect(){
	refreshTimer.stop();
	robot->closeHokuyo();
}
