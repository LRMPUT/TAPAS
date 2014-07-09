/*
 * Constraints.cpp
 *
 *  Created on: 09-07-2014
 *      Author: jachu
 */

#include "Constraints.h"
//Qt
#include <QtGui/QPainter>
#include <QtGui/QColor>
#include <QtGui/QPixmap>
//OpenCV
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Constraints::Constraints(Ui::TrobotQtClass* iui, Debug* idebug) :
	ui(iui),
	debug(idebug)
{
	QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(updateViews()));
	timer.setInterval(100);
	timer.start();
}

Constraints::~Constraints(){

}

void Constraints::updateCameraView(){
	Mat image;
	vector<Point2f> pointCloudCamera = debug->getPointCloudCamera(image);

	cvtColor(image, image, CV_BGR2RGB);
	Mat resizedImage;
	cv::resize(image, resizedImage, cv::Size(ui->constraintCameraViewLabel->width(), ui->constraintCameraViewLabel->height()));
	double scaleX = (double)image.cols / ui->constraintCameraViewLabel->width();
	double scaleY = (double)image.rows / ui->constraintCameraViewLabel->height();
	QPixmap map = QPixmap::fromImage(QImage(resizedImage.ptr(), resizedImage.cols, resizedImage.rows, QImage::Format_RGB888));

	QPainter painter(&map);
	painter.setPen(Qt::red);
	for(int p = 0; p < pointCloudCamera.size(); p++){
		if(pointCloudCamera[p].x >= 0 && pointCloudCamera[p].x < image.cols &&
				pointCloudCamera[p].y >= 0 && pointCloudCamera[p].x < image.rows)
		{
			painter.drawPoint(pointCloudCamera[p].x / scaleX, pointCloudCamera[p].y / scaleY);
		}
	}
	painter.end();

	ui->constraintCameraViewLabel->setPixmap(map);
	ui->constraintCameraViewLabel->update();
}

void Constraints::updateMapView(){
	//TODO Uzupełnić
}

void Constraints::updateViews(){
	if(ui->constraintCameraViewLabel->isVisible()){
		updateCameraView();
	}
	if(ui->constraintMapViewLabel->isVisible()){
		updateMapView();
	}
}
