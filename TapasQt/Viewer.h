/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef VIEWER_H_
#define VIEWER_H_

//OpenCV
#include <opencv2/opencv.hpp>
//QGLViewer
#include <QGLViewer/qglviewer.h>
#include <QGLViewer/camera.h>

class Viewer : public QGLViewer
{
	Q_OBJECT

private:
	cv::Mat posMapCenterGlobal;
	cv::Mat posImuMapCenter;
	cv::Mat pointCloudMapCenter;
	cv::Mat cameraOrigImu;
	cv::Mat imuOrigRobot;
	cv::Mat constraintsMap;
	std::vector<float> vecFieldHist;
	float goalDirection;
	float bestDirection;

	void drawRobot();
	void drawPointCloud();
	void drawConstraintsMap();
	void drawVecFieldHist();

	void multCurMatrix(cv::Mat trans);

	void sphere(float x, float y, float z, float radius);
	void cylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions);

public:
	void rysuj();
	void updatePointCloud(cv::Mat newPointCloud);
	void updateRobotPos(cv::Mat newRobotPos, cv::Mat newMapCenterGlobal);
	void updateConstraintsMap(cv::Mat newConstraintsMap);
	void updateVecFieldHist(std::vector<float> newVecFieldHist, float newGoalDirection, float newBestDirection);
	//void updateCameraOrigImu(cv::Mat newCameraOrigImu);
	//void updateImuOrigGlobal(cv::Mat newImuOrigGlobal);
	Viewer(cv::Mat iImuOrigGlobal, cv::Mat icameraOrigImu);
	~Viewer();

protected:
	virtual void draw();
	virtual void init();
};


#endif /* VIEWER_H_ */
