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

	Mat cameraOrigImu, cameraOrigLaser, imuOrigGlobal;
	debug->getTransformationMatrices(imuOrigGlobal, cameraOrigLaser, cameraOrigImu);
	viewer = new Viewer(imuOrigGlobal, cameraOrigImu);
	ui->constraintMapViewScrollArea->setWidget(viewer);

	//timer.setInterval(100);
	timer.start(100);
}

Constraints::~Constraints(){

}

void Constraints::updateCameraView(){
	//cout << "Updating camera view" << endl;
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
	Mat pointCloudImuMapCenter = debug->getPointCloudImu(curPosImuMapCenter);
	Mat constraintsMap = debug->getMovementConstraints();

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
	//cout << "curPosImuMapCenter.size() = " << curPosImuMapCenter.size() << endl;
	if(!pointCloudImuMapCenter.empty()){
		viewer->updatePointCloud(pointCloudImuMapCenter);
	}
	if(!curPosImuMapCenter.empty()){
		//cout << "Updating constraintsMap" << endl;
		viewer->updateRobotPos(curPosImuMapCenter);
		viewer->updateConstraintsMap(constraintsMap);
		viewer->rysuj();
	}
}

void Constraints::updateClassificationView(){
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
	double scaleY = (globalPlan.maxY - globalPlan.minY)/ui->planningGlobalViewLabel->height();
//	printf("Scales: %f %f\n", scaleX, scaleY);
	QPainter painter(&map);
//	printf("Edges size: %d \n", globalPlan.edges.size());
	for(int e = 0; e < globalPlan.edges.size(); e++){
		int x1 = (globalPlan.edges[e].x1 - globalPlan.minX)/scaleX;
		int y1 = (globalPlan.edges[e].y1 - globalPlan.minY)/scaleY;
		int x2 = (globalPlan.edges[e].x2 - globalPlan.minX)/scaleX;
		int y2 = (globalPlan.edges[e].y2 - globalPlan.minY)/scaleY;
		if(globalPlan.edges[e].isChoosen == true){
			painter.setPen(Qt::red);
		}
		else if(e == globalPlan.curEdge){
			painter.setPen(Qt::green);
		}
		else{
			painter.setPen(Qt::blue);
		}
//		printf("drawLine: %d %d %d %d \n", x1, y1, x2, y2);
		painter.drawLine(x1, y1, x2, y2);
		painter.setPen(Qt::black);
		painter.drawEllipse(x1, y1, 2, 2);
		painter.drawEllipse(x2, y2, 2, 2);
	}
	painter.setPen(Qt::green);
	int rX = (globalPlan.robotX - globalPlan.minX)/scaleX;
	int rY = (globalPlan.robotY - globalPlan.minY)/scaleY;
	painter.drawEllipse(rX, rY, 2, 2);
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
