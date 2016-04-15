/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology

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

QtHokuyo::QtHokuyo(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug) :
	ui(iui),
	robot(irobot),
	debug(idebug)
{
	QObject::connect(ui->hokuyoConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->hokuyoDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));

	refreshTimer.setInterval(100);
	refreshTimer.start();

	map = QPixmap(ui->calibLaserLabel->width(), ui->calibLaserLabel->height());
	openHokuyoClient = nh.serviceClient<TAPAS::PointCloud>("open_hokuyo");
	closeHokuyoClient = nh.serviceClient<std_srvs::Empty>("close_hokuyo");
}

QtHokuyo::~QtHokuyo(){
}

cv::Mat QtHokuyo::getData(){
	return debug->getHokuyoData();
}

void QtHokuyo::refresh(){
//	cout << "QtHokuyo::refresh(), robot->isHokuyoOpen() = " << robot->isHokuyoOpen() << endl;
	if(debug->isHokuyoActive()){
		Mat data = getData();

		map.fill(Qt::white);
		QPainter painter(&map);
		painter.setPen(Qt::blue);

		float scale;
		const int range = 4000;
		scale = (float)min(ui->calibLaserLabel->width(), ui->calibLaserLabel->height()) / (2 * range);

		int origX = ui->calibLaserLabel->width()/2;
		int origY = ui->calibLaserLabel->height()/2;

		QVector<QPointF> lines;
		for(int i = 0; i < data.cols; i++){
//			cout << "Point " << i << " = (" << data.at<int>(0, i) << ", " << data.at<int>(1, i) << ")" << endl;
			lines.append(QPointF(origX - data.at<int>(1, i)*scale, origY - data.at<int>(0, i)*scale));
		}
		painter.drawPolyline(lines);

		painter.end();
		ui->calibLaserLabel->setPixmap(map);
		ui->calibLaserLabel->update();
	}
}

void QtHokuyo::connect(){
	if(ui->hokuyoPortCombo->count() != 0){
		TAPAS::OpenPort srv;
		srv.request.port = ui->hokuyoPortCombo->currentText().toAscii().data();
		openHokuyoClient.call(srv);
	}
}

void QtHokuyo::disconnect(){
	std_srvs::Empty srv;
	closeHokuyoClient.call(srv);
}
