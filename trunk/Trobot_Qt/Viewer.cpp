/*
 * Viewer.cpp
 *
 *  Created on: 11-07-2014
 *      Author: jachu
 */

#include "Viewer.h"

#include <GL/glu.h>
//TAPAS
#include "../Robot/Robot.h"

using namespace std;
using namespace cv;

void Viewer::drawRobot(){
	//cout << "drawRobot()" << endl;
	//cout << "posImuMapCenter.size()" << posImuMapCenter.size() << endl;
	if(!posImuMapCenter.empty()){
		glPushMatrix();
		multCurMatrix(posImuMapCenter);

		glColor3f(0.0f,1.0f,0.0f);
		sphere(0, 0, 0, 200);
		drawAxis(1000);

		glPopMatrix();
	}
	//cout << "End drawRobot()" << endl;
}

void Viewer::drawPointCloud(){
	cout << "drawPointCloud()" << endl;
	glPointSize(5);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for(int p = 0; p < pointCloudMapCenter.cols; p++){
		glVertex4f(pointCloudMapCenter.at<float>(0, p),
					pointCloudMapCenter.at<float>(1, p),
					pointCloudMapCenter.at<float>(2, p),
					pointCloudMapCenter.at<float>(3, p));
	}
	glEnd();

	cout << "End drawPointCloud()" << endl;
}

void Viewer::drawConstraintsMap(){

}


void Viewer::multCurMatrix(cv::Mat trans){
	glMultTransposeMatrixf((float*)trans.data);
}

void Viewer::sphere(float x, float y, float z, float radius){
  GLUquadricObj *quadric=gluNewQuadric();
  gluQuadricNormals(quadric, GLU_SMOOTH);

  glPushMatrix();
  glTranslatef(x, y, z);
  gluSphere(quadric, radius, 30, 30);
  glPopMatrix();

  gluDeleteQuadric(quadric);
}

//kod zaczerpniety ze strony:
//http://lifeofaprogrammergeek.blogspot.com/2008/07/rendering-cylinder-between-two-points.html

void Viewer::cylinder(float x1, float y1, float z1, float x2,float y2, float z2, float radius,int subdivisions){
  GLUquadricObj *quadric=gluNewQuadric();
  gluQuadricNormals(quadric, GLU_SMOOTH);

  float vx = x2-x1;
  float vy = y2-y1;
  float vz = z2-z1;

  if(vz == 0)
      vz = .0001;

  float v = sqrt( vx*vx + vy*vy + vz*vz );
  float ax = 57.2957795*acos( vz/v );
  if ( vz < 0.0 )
      ax = -ax;
  float rx = -vy*vz;
  float ry = vx*vz;
  glPushMatrix();

  glTranslatef( x1,y1,z1 );
  glRotatef(ax, rx, ry, 0.0);
  gluQuadricOrientation(quadric,GLU_OUTSIDE);
  gluCylinder(quadric, radius, radius, v, subdivisions, 1);

  gluQuadricOrientation(quadric,GLU_INSIDE);
  gluDisk( quadric, 0.0, radius, subdivisions, 1);
  glTranslatef( 0,0,v );

  gluQuadricOrientation(quadric,GLU_OUTSIDE);
  gluDisk( quadric, 0.0, radius, subdivisions, 1);
  glPopMatrix();

  gluDeleteQuadric(quadric);
}

void Viewer::draw()
{
	//cout << "rysuje" << endl;
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	glColor3f(1.0f,1.0f,1.0f);
	//camera()->draw();
	drawGrid(MAP_SIZE*MAP_RASTER_SIZE, MAP_SIZE);
	drawAxis(1000);
	glColor3f(1.0, 0, 0);

	drawRobot();
	drawPointCloud();

}

void Viewer::init()
{
	//glClearColor(1.0, 0, 0, 0);
	cout << "inicjuje kamere" << endl;
	camera()->setType(qglviewer::Camera::PERSPECTIVE);
	camera()->setPosition(qglviewer::Vec(-2000, -2000, -2000));
	//camera()->setFieldOfView(3.14/2);
	setSceneRadius(10000);
	camera()->setUpVector(qglviewer::Vec(0.57735, 0.57735, -0.57735));
	camera()->lookAt(sceneCenter());
	cout << "zNear = " << camera()->zNear() << ", zFar = " << camera()->zFar() << endl;
	cout << "scene center = (" << sceneCenter().x << ", " << sceneCenter().y << ", " << sceneCenter().z << ")" << endl;
	cout << "camera position = (" << camera()->position().x << ", " << camera()->position().y << ", " << camera()->position().z << ")" << endl;
	cout << "camera orientation = (" << camera()->viewDirection().x << ", " << camera()->viewDirection().y << ", " << camera()->viewDirection().z << ")" << endl;;
	cout << "wymiary: " << width() << "x" << height() << endl;
}

void Viewer::updatePointCloud(cv::Mat newPointCloud){
	pointCloudMapCenter = newPointCloud;
}

void Viewer::updateRobotPos(cv::Mat newRobotPos){
	posImuMapCenter = newRobotPos;
	//cout << "posImuMapCenter.size()" << posImuMapCenter.size() << endl;
}

void Viewer::updateConstraintsMap(cv::Mat newConstraintsMap){
	constraintsMap = newConstraintsMap;
}

Viewer::Viewer() {

}

Viewer::~Viewer()
{
	//cout << "start of ~Viewer()" << endl;

	//cout << "end of ~Viewer()" << endl;
}

void Viewer::rysuj()
{
	repaint();
}

