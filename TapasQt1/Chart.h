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
