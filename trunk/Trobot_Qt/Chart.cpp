#include <iostream>
#include <vector>
#include <QtGui/QPainter>
#include "Chart.h"

using namespace std;

Chart::Chart(int ichannels, int isaPerSecond, int bufferLength) :
	QPixmap(CHART_RES_X, CHART_RES_Y), channels(ichannels), saPerSecond(isaPerSecond)
{
	data.resize(channels);
	scales.resize(channels);
	colors.resize(channels);
	for(int i = 0; i < channels; i++){
		data[i].resize(bufferLength);
		scales[i] = 1;
		//color
	}
	pos = 0;
	lastDrawn = -1;
	origX = 10;
	origY = CHART_RES_Y;
	scaleT = 0.1;
	fill();
}

void Chart::drawLine(int chan, int first, int last){
	if(origX + first/(saPerSecond*scaleT) < 0){
		first = -origX*scaleT*saPerSecond;
	}
	if(origX + last/(saPerSecond*scaleT) >= CHART_RES_X){
		last = (CHART_RES_X - origX)*scaleT*saPerSecond;
	}
	if(first >= last){
		return;
	}
	//cout << "Updating from " << first << " to " << last << " for channel " << chan << ", scale " << scales[chan] << endl;
	QPoint* polyline = new QPoint[last - first + 1];
	for(int i = first; i <= last; i++){
		//cout << "Punkt (" << origX + i/(scaleT*saPerSecond) << " " << origY + data[chan][i]/scales[chan] << ")" << endl;
		polyline[i - first] = QPoint(origX + i/(saPerSecond*scaleT), origY + data[chan][i]/scales[chan]);
	}
	QPainter painter(this);
	painter.setPen(colors[chan]);
	painter.drawPolyline(polyline, last - first + 1);
	painter.end();
	delete[] polyline;
}

QPoint Chart::getOrigin(){
	return QPoint(origX, origY);
}

void Chart::setOrigin(QPoint newOrig){
	origX = newOrig.x();
	origY = newOrig.y();
}

void Chart::setTimeScale(float newScale){
	scaleT = newScale;
}

float Chart::getScale(int chan){
	return scales[chan];
}

void Chart::setScale(int chan, float newScale){
	scales[chan] = newScale;
}


void Chart::setColor(int chan, QColor newColor){
	colors[chan] = newColor;
}

void Chart::addData(const vector<vector<float> >& newData){
	if(pos + newData[0].size() > data[0].size()){
		throw "Buffer overload";
		return;
	}
	//cout << "Adding data, newData.size() = " << newData.size() << endl;
	//cout << "data.size() = " << data.size() << endl;
	for(int ch = 0; ch < channels; ch++){
		for(int i = 0; i < newData[0].size(); i++){
			//cout << "Dodaje wartosc " << newData[ch][i] << " na pozycji " << pos + i << ", w kanale " << ch << endl;
			data[ch][pos + i] = newData[ch][i];
		}
	}
	pos += newData[0].size();
	//cout << "pos = " << pos << endl;
}

void Chart::updateFromLast(){
	for(int ch = 0; ch < channels; ch++){
		drawLine(ch, lastDrawn, pos - 1);
	}
	lastDrawn = pos - 1;
}

void Chart::repaint(){
	this->fill(0xffffffff);
	lastDrawn = -1;
	updateFromLast();
}

void Chart::clear(){
	this->fill(0xffffffff);
	lastDrawn = -1;
	pos = 0;
}
