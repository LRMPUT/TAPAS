/*
 * Viewer.h
 *
 *  Created on: 11-07-2014
 *      Author: jachu
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
	cv::Mat posImuMapCenter;
	cv::Mat pointCloudMapCenter;
	cv::Mat constraintsMap;

	void drawRobot();
	void drawPointCloud();
	void drawConstraintsMap();

	void multCurMatrix(cv::Mat trans);

	void sphere(float x, float y, float z, float radius);
	void cylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions);

public:
	void rysuj();
	void updatePointCloud(cv::Mat newPointCloud);
	void updateRobotPos(cv::Mat newRobotPos);
	void updateConstraintsMap(cv::Mat newConstraintsMap);
	Viewer();
	~Viewer();

protected:
	virtual void draw();
	virtual void init();
};


#endif /* VIEWER_H_ */
