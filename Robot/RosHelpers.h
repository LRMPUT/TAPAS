#ifndef ROSHELPERS_H_
#define ROSHELPERS_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "TAPAS/Matrix.h"
//OpenCV
#include <opencv2/opencv.hpp>

class RosHelpers {
public:

	static TAPAS::Matrix makeMatrixMsg(cv::Mat matrix);

	static cv::Mat readMatrixMsg(TAPAS::Matrix matrix);

	static cv::Mat readImageMsg(const sensor_msgs::ImageConstPtr& msg);

	static cv::Mat readMatrixParam(ros::NodeHandle nh, const char* param, int rows, int cols);
};

#endif /* ROSHELPERS_H_ */
