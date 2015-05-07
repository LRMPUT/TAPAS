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

#include "Constraints.h"
//Qt
#include <QtGui/QPainter>
#include <QtGui/QColor>
#include <QtGui/QPixmap>
//OpenCV
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Constraints::Constraints(Ui::TapasQtClass* iui, Debug* idebug) :
	ui(iui),
	debug(idebug)
{
	QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(updateViews()));

	Mat cameraOrigImu, cameraOrigLaser, imuOrigRobot;
	debug->getTransformationMatrices(imuOrigRobot, cameraOrigLaser, cameraOrigImu);
	viewer = new Viewer(imuOrigRobot, cameraOrigImu);
	ui->constraintMapViewScrollArea->setWidget(viewer);

	//timer.setInterval(100);
	timer.start(100);
}

Constraints::~Constraints(){

}

void Constraints::updateCameraView(){
//	cout << "Updating camera view" << endl;
	Mat image;
	vector<Point2f> pointCloudCamera = debug->getPointCloudCamera(image);
	//cout << "Got point cloud camera" << endl;
	QPixmap map;
	double scaleX = 640 / ui->constraintCameraViewLabel->width();
	double scaleY = 480 / ui->constraintCameraViewLabel->height();

	if(!image.empty()){
		//cout << "image.rows = " << image.rows << ", image.col = " << image.cols << endl;
		//cout << ui->constraintCameraViewLabel->width() << ", " << ui->constraintCameraViewLabel->height() << endl;
		cvtColor(image, image, CV_BGR2RGB);
		Mat resizedImage;
		cv::resize(image, resizedImage, cv::Size(ui->constraintCameraViewLabel->width(), ui->constraintCameraViewLabel->height()));
		scaleX = (double)image.cols / ui->constraintCameraViewLabel->width();
		scaleY = (double)image.rows / ui->constraintCameraViewLabel->height();
		map = QPixmap::fromImage(QImage(resizedImage.ptr(), resizedImage.cols, resizedImage.rows, QImage::Format_RGB888));
	}
	else{
		map = QPixmap(ui->constraintCameraViewLabel->width(), ui->constraintCameraViewLabel->height());
		map.fill(Qt::white);
	}

	QPainter painter(&map);
	painter.setPen(Qt::red);
	for(int p = 0; p < pointCloudCamera.size(); p++){
		if(pointCloudCamera[p].x >= 0 && pointCloudCamera[p].x < image.cols &&
				pointCloudCamera[p].y >= 0 && pointCloudCamera[p].y < image.rows)
		{
			painter.drawPoint(pointCloudCamera[p].x / scaleX, pointCloudCamera[p].y / scaleY);
		}
	}
	painter.end();

	ui->constraintCameraViewLabel->setPixmap(map);
	ui->constraintCameraViewLabel->update();
}

void Constraints::updateMapView(){
	//cout << "updateMapView()" << endl;
	Mat curPosImuMapCenter;
	Mat posMapCenterGlobal;
	Mat pointCloudImuMapCenter = debug->getPointCloudImu(curPosImuMapCenter, posMapCenterGlobal);
	Mat constraintsMap = debug->getMovementConstraints();
	vector<float> vecFieldHist;
	float goalDirection;
	float bestDirection;
	debug->getVecFieldHist(vecFieldHist, goalDirection, bestDirection);
	//cout << "vecFieldHist.size() = " << vecFieldHist.size() << endl;
	float imuAccVariance = debug->getImuAccVariance();

	stringstream tmp;
	tmp.width(4);
	tmp.precision(4);
	tmp.setf(std::ios::right, std::ios::adjustfield);
	for(int r = 0 ;r < curPosImuMapCenter.rows; r++){
		for(int c = 0; c < curPosImuMapCenter.cols; c++){
			tmp << curPosImuMapCenter.at<float>(r, c) << " ";
		}
		tmp << endl;
	}
	//tmp << curPosImuMapCenter;
	ui->constraintCurPosLabel->setText(QString(tmp.str().c_str()));
	ui->constraintImuAccVarianceLabel->setText(QString("%1").arg(imuAccVariance, 5, 'f'));
	//cout << "curPosImuMapCenter.size() = " << curPosImuMapCenter.size() << endl;
	if(!pointCloudImuMapCenter.empty()){
		viewer->updatePointCloud(pointCloudImuMapCenter);
	}
	if(!vecFieldHist.empty()){
		viewer->updateVecFieldHist(vecFieldHist, goalDirection, bestDirection);
	}
	if(!curPosImuMapCenter.empty() && !posMapCenterGlobal.empty()){
		//cout << "Updating constraintsMap" << endl;
		viewer->updateRobotPos(curPosImuMapCenter, posMapCenterGlobal);
		viewer->updateConstraintsMap(constraintsMap);
		viewer->rysuj();
	}
}

void Constraints::updateClassificationView(){
//	cout << "Constraints::updateClassificationView()" << endl;
	Mat image = debug->getClassifiedImage();
	QPixmap map;
	double scaleX = 640 / ui->constraintClassificationViewLabel->width();
	double scaleY = 480 / ui->constraintClassificationViewLabel->height();

	if(!image.empty()){
		cvtColor(image, image, CV_BGR2RGB);
		Mat resizedImage;
		cv::resize(image, resizedImage, cv::Size(ui->constraintClassificationViewLabel->width(), ui->constraintClassificationViewLabel->height()));
		scaleX = (double)image.cols / ui->constraintClassificationViewLabel->width();
		scaleY = (double)image.rows / ui->constraintClassificationViewLabel->height();
		map = QPixmap::fromImage(QImage(resizedImage.ptr(), resizedImage.cols, resizedImage.rows, QImage::Format_RGB888));
		ui->constraintClassificationViewLabel->setPixmap(map);
		ui->constraintClassificationViewLabel->update();
	}
}

void Constraints::updateGlobalPlanView(){
//	printf("Update Global Plan View\n");
	GlobalPlanner::GlobalPlanInfo globalPlan = debug->getGlobalPlan();
	QPixmap map(ui->planningGlobalViewLabel->width(), ui->planningGlobalViewLabel->height());


	map.fill(Qt::white);

	double scaleX = (globalPlan.maxX - globalPlan.minX)/ui->planningGlobalViewLabel->width();
	double scaleY = (globalPlan.maxY - globalPlan.minY)
			/ ui->planningGlobalViewLabel->height();
	double scale = scaleX > scaleY ? scaleX : scaleY;
//	printf("Scales: %f %f and edges size: %d, curEdge : %d\n", scaleX, scaleY,
//			globalPlan.edges.size(), globalPlan.curEdge);
	QPainter painter(&map);

	std::set<GlobalPlanner::Edge>::iterator it = globalPlan.edges.begin();
	int e=0;
	for(; it != globalPlan.edges.end(); ++it,e++){
		int x1 = (it->x1 - globalPlan.minX)/scale;
		int y1 = (it->y1 - globalPlan.minY)/scale;
		int x2 = (it->x2 - globalPlan.minX)/scale;
		int y2 = (it->y2 - globalPlan.minY)/scale;
		if (e == globalPlan.curEdge) {
//			printf("Drawing current edge !!! Number: %d || %d %d %d %d\n", e, x1, y1, x2, y2);
			QPen myPen(Qt::red, 7, Qt::SolidLine);
			painter.setPen(myPen);
		} else if (it->isChosen == true) {
			QPen myPen(Qt::green, 2, Qt::SolidLine);
			painter.setPen(myPen);
		}
		else{
			QPen myPen(Qt::blue, 2, Qt::SolidLine);
			painter.setPen(myPen);
		}
//		printf("drawLine: %d %d %d %d \n", x1, y1, x2, y2);
		painter.drawLine(x1, y1, x2, y2);
		QPen myPen(Qt::black, 2, Qt::SolidLine);
		painter.setPen(myPen);
		painter.drawEllipse(x1, y1, 2, 2);
		painter.drawEllipse(x2, y2, 2, 2);
	}

	// Robot coordinates
	painter.setPen(Qt::green);
//	printf("Drawing robot's coordinates: %f %f\n",globalPlan.robotX,globalPlan.robotY);
	int rX = (globalPlan.robotX - globalPlan.minX)/scale;
	int rY = (globalPlan.robotY - globalPlan.minY)/scale;
	painter.setBrush(Qt::green);
	painter.drawEllipse(rX, rY, 6, 6);

	painter.setPen(Qt::red);
//	printf("Drawing goal's coordinates: %f %f\n",globalPlan.goalX,globalPlan.goalY);
	int gX = (globalPlan.goalX - globalPlan.minX)/scale;
	int gY = (globalPlan.goalY - globalPlan.minY)/scale;
	painter.setBrush(Qt::red);
	painter.drawEllipse(gX, gY, 6, 6);


	painter.end();
	ui->planningGlobalViewLabel->setPixmap(map);
	ui->planningGlobalViewLabel->update();
}

void Constraints::updateViews(){
	//cout << "Updating views" << endl;
	if(ui->constraintCameraViewLabel->isVisible()){
		updateCameraView();
	}
	if(ui->constraintMapViewScrollArea->isVisible()){
		updateMapView();
	}
	if(ui->constraintClassificationViewLabel->isVisible()){
		updateClassificationView();
	}
	if(ui->planningGlobalViewLabel->isVisible()){
		updateGlobalPlanView();
	}
}
