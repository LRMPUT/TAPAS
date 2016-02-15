#include "ConstraintsHelpers.h"

using namespace cv;
using namespace std;

cv::Mat ConstraintsHelpers::compOrient(cv::Mat imuData){
	//cout << "Computing orientation from IMU" << endl;
	//cout << "imuData = " << imuData << endl;

	Mat ret(Mat::eye(4, 4, CV_32FC1));
	float yaw = imuData.at<float>(2, 3)*PI/180;
	float pitch = imuData.at<float>(1, 3)*PI/180;
	float roll = imuData.at<float>(0, 3)*PI/180;
	//cout << "Computing Rz, Ry, Rx, yaw = " << yaw << endl;
	Matx33f Rz(	cos(yaw), -sin(yaw), 0,
				sin(yaw), cos(yaw), 0,
				0, 0, 1);
	//cout << "Rz = " << Rz << endl;
	Matx33f Ry(	cos(pitch), 0, sin(pitch),
				0, 1, 0,
				-sin(pitch), 0, cos(pitch));
	//cout << "Ry = " << Ry << endl;
	Matx33f Rx(	1, 0, 0,
				0, cos(roll), -sin(roll),
				0, sin(roll), cos(roll));
	//cout << "Rx = " << Rx << endl;
	Mat tmp(Rz*Ry*Rx);
	tmp.copyTo(ret(Rect(0, 0, 3, 3)));

	//cout << "End computing orientation from IMU" << endl;
	return ret;
}


cv::Mat ConstraintsHelpers::compTrans(	cv::Mat orient,
							cv::Mat encodersDiff,
							ros::NodeHandle &nh)
{
	float wheelCir, wheelDistance;
	int encodersCPR;

	nh.getParam("wheelCir", wheelCir);
	nh.getParam("encodersCPR", encodersCPR);
	nh.getParam("wheelDistance", wheelDistance);

	wheelCir *= PI;

	float sl = (float)encodersDiff.at<int>(0)*wheelCir/encodersCPR;
	float sr = (float)encodersDiff.at<int>(1)*wheelCir/encodersCPR;

	if(fabs(sl) > 10000 || fabs(sr) > 10000){
		sl = 0.0f;
		sr = 0.0f;
	}

	float theta = (sl - sr)/(-wheelDistance);
	Mat trans(4, 1, CV_32FC1, Scalar(0));
	//cout << "theta = " << theta << endl;
	if(theta < 0.1){
		trans.at<float>(0) = (sl + sr)/2;
	}
	else{
		float r = -wheelDistance*(sl - sr)/(2*(sl - sr));
		trans.at<float>(0) = r*sin(theta);
		trans.at<float>(1) = r*(cos(theta) - 1);

	}
	//cout << "transC = " << trans << endl << "orientC = " << orient << endl;
	trans = orient*trans;
	return trans;
}

cv::Mat ConstraintsHelpers::compNewPos(cv::Mat lprevImu, cv::Mat lcurImu,
									cv::Mat lprevEnc, cv::Mat lcurEnc,
									cv::Mat lposOrigMapCenter,
									cv::Mat lmapCenterOrigGlobal,
									ros::NodeHandle &nh)
{
	Mat ret = Mat::eye(4, 4, CV_32FC1);
	if(!lposOrigMapCenter.empty() && !lmapCenterOrigGlobal.empty()){
		//cout << "compOrient(lcurImu) = " << compOrient(lcurImu) << endl;
		//cout << "lmapCenterGlobal = " << lmapCenterGlobal << endl;
		//cout << "Computing curPos" << endl;
		//cout << "encodersCur - encodersPrev = " << encodersCur - encodersPrev << endl;
		Mat trans = lmapCenterOrigGlobal.inv()*compTrans(compOrient(lprevImu), lcurEnc - lprevEnc, nh);
		//cout << "trans = " << trans << endl;
		//cout << "Computing curTrans" << endl;
		Mat curTrans = Mat(lposOrigMapCenter, Rect(3, 0, 1, 4)) + trans;
		//cout << "Computing curRot" << endl;

		Mat curRot = lmapCenterOrigGlobal.inv()*compOrient(lcurImu);
		//cout << "curRot = " << curRot << endl;

		curRot.copyTo(ret);
		curTrans.copyTo(Mat(ret, Rect(3, 0, 1, 4)));
	}
	return ret;
}

void ConstraintsHelpers::processPointCloud(cv::Mat hokuyoData,
											cv::Mat& pointCloudOrigMapCenter,
											std::queue<PointsPacket>& pointsInfo,
											std::chrono::high_resolution_clock::time_point hokuyoTimestamp,
											std::chrono::high_resolution_clock::time_point curTimestamp,
											cv::Mat curPosOrigMapCenter,
											std::mutex& mtxPointCloud,
											cv::Mat cameraOrigLaser,
											cv::Mat cameraOrigImu,
											ros::NodeHandle &nh)
{
	Mat hokuyoCurPoints(6, hokuyoData.cols, CV_32FC1);
	int minLaserDist, pointCloudTimeout;

	nh.getParam("minLaserDist", minLaserDist);
	nh.getParam("pointCloudTimeout", pointCloudTimeout);

	//cout << "Copying hokuyo data" << endl;
	int countPoints = 0;
	for(int c = 0; c < hokuyoData.cols; c++){
		//cout << hokuyoData.at<int>(2, c) << endl;
		if(hokuyoData.at<int>(2, c) > minLaserDist){
			hokuyoCurPoints.at<float>(0, countPoints) = -hokuyoData.at<int>(1, c);
			hokuyoCurPoints.at<float>(1, countPoints) = 0.0;
			hokuyoCurPoints.at<float>(2, countPoints) = hokuyoData.at<int>(0, c);
			hokuyoCurPoints.at<float>(3, countPoints) = 1.0;
			hokuyoCurPoints.at<float>(4, countPoints) = hokuyoData.at<int>(2, c);
			hokuyoCurPoints.at<float>(5, countPoints) = hokuyoData.at<int>(3, c);
			countPoints++;
		}
	}
	hokuyoCurPoints = hokuyoCurPoints.colRange(0, countPoints);
//	cout << "hokuyoCurPoints.size() = " << hokuyoCurPoints.size() << endl;
//	cout << "hokuyoData.size() = " << hokuyoData.size() << endl;

	//cout << "Removing old points" << endl;
	//remove all points older than pointCloudTimeout ms
	int pointsSkipped = 0;
	if(pointsInfo.size() > 0){
		//cout << "dt = " << std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsQueue.front().timestamp).count() << endl;
		while(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsInfo.front().timestamp).count() > pointCloudTimeout){
			pointsSkipped += pointsInfo.front().numPoints;
			pointsInfo.pop();
			if(pointsInfo.size() == 0){
				break;
			}
		}
	}

	std::unique_lock<std::mutex> lck(mtxPointCloud);
	//cout << "Moving pointCloudOrigMapCenter, pointsSkipped = " << pointsSkipped << endl;
	Mat tmpAllPoints(hokuyoCurPoints.rows, pointCloudOrigMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped, CV_32FC1);
	if(!pointCloudOrigMapCenter.empty()){
		//cout << "copyTo" << endl;
		if(pointsSkipped <	pointCloudOrigMapCenter.cols){
			pointCloudOrigMapCenter.colRange(pointsSkipped,
												pointCloudOrigMapCenter.cols).
							copyTo(tmpAllPoints.colRange(0, pointCloudOrigMapCenter.cols - pointsSkipped));
		}
	}
//	if(debugLevel >= 1){
//		cout << "countPoints = " << countPoints << ", hokuyoCurPoints.size = " << hokuyoCurPoints.size() << endl;
//	}
	if(countPoints > 0){

		//cout << "Moving to camera orig" << endl;
		hokuyoCurPoints.rowRange(0, 4) = cameraOrigLaser.inv()*hokuyoCurPoints.rowRange(0, 4);
		//cout << "Addding hokuyoCurPoints" << endl;
//		if(debugLevel >= 1){
//			cout << "Addding hokuyoCurPoints" << endl;
//		}
		Mat curPointCloudOrigMapCenter(hokuyoCurPoints.rows, hokuyoCurPoints.cols, CV_32FC1);
		Mat tmpCurPoints = curPosOrigMapCenter*cameraOrigImu*hokuyoCurPoints.rowRange(0, 4);
		tmpCurPoints.copyTo(curPointCloudOrigMapCenter.rowRange(0, 4));
		hokuyoCurPoints.rowRange(4, 6).copyTo(curPointCloudOrigMapCenter.rowRange(4, 6));
		//cout << hokuyoCurPointsGlobal.channels() << ", " << hokuyoAllPointsGlobal.channels() << endl;
//		cout << pointCloudOrigMapCenter.size() << " " << curPointCloudCameraMapCenter.size() << " " << tmpAllPoints.size() << endl;
//		if(debugLevel >= 1){
//			cout << "pointsSkipped = " << pointsSkipped << endl;
//		}
		curPointCloudOrigMapCenter.copyTo(tmpAllPoints.colRange(pointCloudOrigMapCenter.cols - pointsSkipped,
																	pointCloudOrigMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped));
//		if(debugLevel >= 1){
//			cout << "curPointCloudCameraMapCenter copied" << endl;
//		}
		pointsInfo.push(PointsPacket(hokuyoTimestamp, hokuyoCurPoints.cols));

		pointCloudOrigMapCenter = tmpAllPoints;
	}
	lck.unlock();
}