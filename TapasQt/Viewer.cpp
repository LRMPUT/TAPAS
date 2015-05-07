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

#include "Viewer.h"

#include <GL/glu.h>
//TAPAS
#include "../Robot/Robot.h"

using namespace std;
using namespace cv;

static const Mat posCameraLaser(Matx44f(
	0.993368616554069,   0.112106788667435,  -0.025511949751658, 45.752060961356,
	  -0.076921557061355,   0.812953576727158,   0.577225741063041, -596.031193651004,
	   0.085450954966652,  -0.571435516940621,   0.816185998577586, -327.210204552425,
	   0,	0,	0,	1));

static const Mat posImuCamera(Matx44f(
	0.010961908195432,   0.998309659916334,  -0.057075909860889,	0,
	-0.64278761,   0.054119665240738,   0.766044443,	0,
	0.766044443,   0.021186900480340,  0.64278761,	0,
	0,	0,	0,	1));

void Viewer::drawRobot(){
	//cout << "drawRobot()" << endl;
	//cout << "posImuMapCenter.size()" << posImuMapCenter.size() << endl;
	if(!posImuMapCenter.empty()){
		glPushMatrix();
		multCurMatrix(posImuMapCenter);

		glPushMatrix();
		multCurMatrix(posMapCenterGlobal.inv());
		drawAxis(1000);
		glPopMatrix();

		glColor3f(1.0f,0.0f,0.0f);
		sphere(0, 0, 0, 200);
		drawAxis(1000);

		multCurMatrix(posImuCamera.inv());
//		drawAxis(1000);

		multCurMatrix(posCameraLaser.inv());
		drawAxis(1000);

		glPopMatrix();
	}
	//cout << "End drawRobot()" << endl;
}

void Viewer::drawPointCloud(){
	//cout << "drawPointCloud()" << endl;
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(0.0f, 1.0f, 0.0f);
	glPointSize(5);
	glDisable(GL_LIGHTING);
	glBegin(GL_POINTS);
	for(int p = 0; p < pointCloudMapCenter.cols; p++){
		glVertex4f(pointCloudMapCenter.at<float>(0, p),
					pointCloudMapCenter.at<float>(1, p),
					pointCloudMapCenter.at<float>(2, p),
					pointCloudMapCenter.at<float>(3, p));
	}
	glEnd();
	glEnable(GL_LIGHTING);
	//cout << "End drawPointCloud()" << endl;
}

void Viewer::drawConstraintsMap(){
	//cout << "drawConstraintsMap()" << endl;
	if(!constraintsMap.empty()){
		//cout << "drawing" << endl;
		glPushMatrix();
		multCurMatrix(imuOrigRobot.inv());

		glDisable(GL_LIGHTING);
		glBegin(GL_QUADS);
		//glColor3f(0.0, 0.0, 1.0);
		for(int y = 0; y < MAP_SIZE; y++){
			for(int x = 0; x < MAP_SIZE; x++){
				glColor4f(0.0, 0.0, 1.0, constraintsMap.at<float>(x, y));
				//cout << "drawing QUAD at (" << x << ", " << y << ")" << endl;
				glVertex3f((x - MAP_SIZE/2) * MAP_RASTER_SIZE, (y - MAP_SIZE/2)*MAP_RASTER_SIZE, 0);
				glVertex3f((x + 1 - MAP_SIZE/2)*MAP_RASTER_SIZE, (y - MAP_SIZE/2)*MAP_RASTER_SIZE, 0);
				glVertex3f((x + 1 - MAP_SIZE/2)*MAP_RASTER_SIZE, (y + 1 - MAP_SIZE/2)*MAP_RASTER_SIZE, 0);
				glVertex3f((x - MAP_SIZE/2)*MAP_RASTER_SIZE, (y + 1 - MAP_SIZE/2)*MAP_RASTER_SIZE, 0);
			}
		}
		glEnd();
		glEnable(GL_LIGHTING);
		glPopMatrix();
	}
	//cout << "End drawConstraintsMap()" << endl;
}

void Viewer::drawVecFieldHist(){
	if(!posImuMapCenter.empty() && !vecFieldHist.empty()){
		glPushMatrix();
		Mat posNoOrient = posImuMapCenter.clone();
		Mat eye33 = Mat::eye(3, 3, CV_32FC1);
		eye33.copyTo(Mat(posNoOrient, Rect(0, 0, 3, 3)));
		multCurMatrix(posNoOrient);
		multCurMatrix(imuOrigRobot.inv());
		glTranslatef(0, 0, -100);	//100 mm over ground;

		int numSect = vecFieldHist.size();
		float alpha = (float)360/numSect;
		float startAngle = 270;

		glDisable(GL_LIGHTING);
		GLUquadricObj *quadric=gluNewQuadric();
		gluQuadricNormals(quadric, GLU_SMOOTH);

		for(int s = 0; s < numSect; s++){
			//cout << "vecFieldHist[" << s << "] = " << vecFieldHist[s] << endl;
			//cout << "alpha = " << min(vecFieldHist[s]/3, 1.0f) << endl;
			glColor4f(0.0, 1.0, 1.0, min(vecFieldHist[s]/3, 1.0f));
			gluPartialDisk(quadric,
							1000,
							1500,
							30,
							30,
							startAngle - s*alpha,
							-alpha);
		}

		gluDeleteQuadric(quadric);
		glEnable(GL_LIGHTING);

		glColor4f(0.0, 1.0, 1.0, 1.0);
		static const float arrowLen = 1000;
		drawArrow(qglviewer::Vec(0, 0, 0),
					qglviewer::Vec(arrowLen*cos(bestDirection*PI/180), arrowLen*sin(bestDirection*PI/180), 0),
					30);


		glTranslatef(0, 0, -50);	//100 mm over ground;
		glColor4f(1.0, 0.0, 1.0, 1.0);
		drawArrow(qglviewer::Vec(0, 0, 0),
					qglviewer::Vec(arrowLen*cos(goalDirection*PI/180), arrowLen*sin(goalDirection*PI/180), 0),
					30);

		glPopMatrix();
	}
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
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//cout << "rysuje" << endl;
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	glColor3f(1.0f,1.0f,1.0f);
	//camera()->draw();
	glPushMatrix();
	multCurMatrix(imuOrigRobot.inv());
	drawGrid(MAP_SIZE*MAP_RASTER_SIZE/2, MAP_SIZE);
	drawAxis(1000);
	glPopMatrix();
	glColor3f(1.0, 0, 0);

	drawRobot();
	drawPointCloud();
	drawConstraintsMap();
	drawVecFieldHist();
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

void Viewer::updateRobotPos(cv::Mat newRobotPos, cv::Mat newMapCenterGlobal){
	posImuMapCenter = newRobotPos;
	posMapCenterGlobal = newMapCenterGlobal;
	//cout << "posImuMapCenter.size()" << posImuMapCenter.size() << endl;
}

void Viewer::updateConstraintsMap(cv::Mat newConstraintsMap){
	constraintsMap = newConstraintsMap;
}


void Viewer::updateVecFieldHist(std::vector<float> newVecFieldHist, float newGoalDirection, float newBestDirection){
	vecFieldHist = newVecFieldHist;
	goalDirection = newGoalDirection;
	bestDirection = newBestDirection;
}

/*void Viewer::updateCameraOrigImu(cv::Mat newCameraOrigImu){
	cameraOrigImu = newCameraOrigImu;
}

void Viewer::updateImuOrigRobot(cv::Mat newImuOrigRobot){
	imuOrigRobot = newImuOrigRobot;
}*/

Viewer::Viewer(cv::Mat iimuOrigRobot, cv::Mat icameraOrigImu) {
	iimuOrigRobot.copyTo(imuOrigRobot);
	icameraOrigImu.copyTo(cameraOrigImu);
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

