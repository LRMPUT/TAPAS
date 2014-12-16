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

#ifndef CHART_H
#define CHART_H

#define CHART_RES_X 640
#define CHART_RES_Y 480

#include <QtGui/QPixmap>
#include <vector>


/*struct Scale {
	Scale(int iscaleX, int iscaleY){
		//units per pixel
		scaleX = iscaleX;
		scaleY = iscaleY;
	}
	int scaleX, scaleY;
};*/

class Chart : public QPixmap
{
	int channels;
	int saPerSecond;
	std::vector<std::vector<float> > data;
	int pos;
	int lastDrawn;
	int origX, origY;
	float scaleT;
	std::vector<float> scales;
	std::vector<QColor> colors;

	void drawLine(int nchan, int first, int last);
public:
	Chart();
	Chart(int ichannels, int isamplesPerSecond, int bufferLength = 524288);
	QPoint getOrigin();
	void setOrigin(QPoint newOrig);
	void setTimeScale(float newScale);
	float getScale(int chan);
	void setScale(int chan, float newScale);
	void setColor(int chan, QColor newColor);
	void addData(const std::vector<std::vector<float> >& newData);
	void updateFromLast();
	void repaint();
	void clear();
};


#endif //CHART_H
