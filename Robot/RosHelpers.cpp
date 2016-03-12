#include "RosHelpers.h"

using namespace std;
using namespace cv;

TAPAS::Matrix RosHelpers::makeMatrixMsg(cv::Mat matrix) {
	TAPAS::Matrix msg;
	msg.rows = matrix.rows;
	msg.cols = matrix.cols;
	msg.data.assign((float*)matrix.datastart, (float*)matrix.dataend);
	return msg;
}

TAPAS::IMU RosHelpers::makeIMUMsg(cv::Mat matrix) {
  	TAPAS::IMU msg;
	memcpy(msg.data.data(), matrix.data, 3*4*sizeof(float));
	return msg;
}

cv::Mat RosHelpers::readMatrixMsg(TAPAS::Matrix msg) {
	cv::Mat mat(msg.rows, msg.cols, CV_32FC1, msg.data.data());
	return mat;
}

cv::Mat RosHelpers::readIMUMsg(TAPAS::IMU msg) {
	cv::Mat mat(3, 4, CV_32FC1, msg.data.data());
	return mat;
}

cv::Mat RosHelpers::readImageMsg(const sensor_msgs::ImageConstPtr& msg) {
	sensor_msgs::Image img = *msg;
	cv::Mat mat = cv::Mat(img.height, img.width, CV_8UC3, img.data.data(), img.step).clone();
	return mat;
}

cv::Mat RosHelpers::readMatrixParam(ros::NodeHandle nh, const char* param, int rows, int cols){
	string paramStr;
	nh.getParam(param, paramStr);

	stringstream tmpStr(paramStr);
	Mat ret = Mat(rows, cols, CV_32FC1);
	for(int row = 0; row < rows; row++){
		for(int col = 0; col < cols; col++){
			float tmpVal;
			tmpStr >> tmpVal;
			ret.at<float>(row, col) = tmpVal;
		}
	}
	return ret;
}