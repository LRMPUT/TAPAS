#include "PlannerHelpers.h"

using namespace cv;
using namespace std;

float PlannerHelpers::rotMatToEulerYaw(Mat rotMat) {

	float R11 = rotMat.at<float>(0, 0);
	float R21 = rotMat.at<float>(1, 0);
	float R31 = rotMat.at<float>(2, 0);

	float teta1 = -asin(R31);
	float teta2 = PI - teta1;
	float fi1 = 0;
	float fi2 = 0;

	if (fabs(R31) >= 1.001 || fabs(R31) < 0.999) {

		fi1 = atan2(R21 / cos(teta1), R11 / cos(teta1));
		fi2 = atan2(R21 / cos(teta2), R11 / cos(teta2));
	} else {
		fi1 = 0.0;
		fi2 = 0.0;
	}

	return fi1;
}

void PlannerHelpers::updateHistogram(std::vector<float>& histSectors,
									cv::Mat posRobotMapCenter,
									cv::Mat constraints,
									Parameters localPlannerParams)
{

	float angPosition;
	float density;

	const float hist_A = 1;
	const float hist_B = hist_A / localPlannerParams.maxDistance;
	/// Get current Constraints Raster Map
	//Mat constraints = robot->getMovementConstraints();
	//Mat posRobotMapCenter = robot->getPosImuConstraintsMapCenter();

	// update position of the robot
	float curRobX = posRobotMapCenter.at<float>(0, 3);
	float curRobY = posRobotMapCenter.at<float>(1, 3);

	//cout<<"RobotX"<<curRobCellX<<endl;
	//cout<<"RobotY"<<curRobCellY<<endl;


	/// for each cell of the raster map
	for (int x = 0; x < MAP_SIZE; x++) {
		for (int y = 0; y < MAP_SIZE; y++) {
			float cellX = ((x + 0.5)  - MAP_SIZE/2)*MAP_RASTER_SIZE;
			float cellY = ((y + 0.5)  - MAP_SIZE/2)*MAP_RASTER_SIZE;
			///angular position
//			angPosition = 180/PI*atan2(float(x - curRobCellX), y - curRobCellY);
			angPosition = atan2(float(cellY - curRobY), float(cellX - curRobX)) * 180 / PI;
			//cout<<"angPosition"<<angPosition<<endl;
			/// value of obstacle vector
			float c = pow(constraints.at<float>(x, y), 2);
			float d = sqrt(pow((float) (cellX - curRobX), 2)+ pow((float) (cellY - curRobY), 2));
			density = c * max(hist_A - hist_B * d, 0.0f);

//			cout << "map (" << x << ", " << y << "), histB = " << hist_B << ", c = " << c << ", d = " << d << ", density = " << density << endl;
			/// update proper sector of histogram
			if (angPosition > 179.999)
				angPosition = -180.0;
			histSectors.at(floor(((angPosition + 180) / localPlannerParams.histResolution))) +=
					density;
		}
	}

//	for (int sect = 0; sect < numHistSectors; sect++) {
//		std::cout << "Angular histogram values for " << sect * localPlannerParams.histResolution - 180
//				<< " is " << histSectors.at(sect) << endl;
//	}
}

void PlannerHelpers::smoothHistogram(std::vector<float>& histSectors,
									Parameters localPlannerParams)
{

	int kernelSize = 2*localPlannerParams.gauss3sig/localPlannerParams.histResolution;
	kernelSize += (kernelSize + 1) % 2;	//kernelSize must be odd
	int kernelMid = kernelSize/2;
	cv::Mat gaussKernel = getGaussianKernel(kernelSize,
											(localPlannerParams.gauss3sig/localPlannerParams.histResolution)/3,
											CV_32FC1);
	gaussKernel *= 1/gaussKernel.at<float>(kernelMid);
	//cout << "gaussKernel = " << gaussKernel << endl;
	vector<float> newHistSectors(histSectors.size(), 0);
	for(int i = 0; i < histSectors.size(); i++){
		for(int k = 0; k < kernelSize; k++){
			newHistSectors[i] += histSectors[(i + k - kernelMid + histSectors.size()) % histSectors.size()]
			                      * gaussKernel.at<float>(k);
		}
	}
	histSectors = newHistSectors;
}


/// return Goal Direction converted from Global Map to
/// local MAP
float PlannerHelpers::determineGoalInLocalMap(cv::Mat posLocalToGlobalMap,
											float goalDirGlobalMap) {

	float globalYaw = 0.0;
	float localToGlobalDifference = 0.0;

	//cout<<"determineGoalInLocalMap"<<endl;

	/// get Orientation of Local Map in Global Map
	//Mat posLocalToGlobalMap = robot->getLocalMapPosInGlobalMap();
	/// and convert this orentation to euler yaw
	globalYaw = rotMatToEulerYaw(posLocalToGlobalMap);
	globalYaw = globalYaw * 180 / PI;
//	cout << "Global_YAW: " << globalYaw << endl;
	// calculate difference between local and global map in orientation
	localToGlobalDifference = goalDirGlobalMap - globalYaw;
//	cout << "Difference: " << localToGlobalDifference << endl;

	//cout<<"determineGoalInLocalMap"<<endl;
	return goalDirGlobalMap - globalYaw;
}

float PlannerHelpers::findOptimSector(const std::vector<float>& histSectors,
									cv::Mat posRobotMapCenter,
									float goalDirLocalMap,
									Parameters localPlannerParams)
{


//	cout << "calculateLocalDirection START" << endl;
	// goal direction+offset/ sectors alpha

	float robotDirLocalMap = rotMatToEulerYaw(posRobotMapCenter);
	robotDirLocalMap = robotDirLocalMap * 180 / PI;

	/// if goal sector is free sector
//	cout << "Free sector size : " << freeSectors.size() << endl;
	int bestSectorID = -1;
	float bestSectorScore = -1;
	for (int i = 0; i < histSectors.size(); i++) {
		//cout << "Free vs goal : " << freeSectors[i] << " " << goalSector
		//		<< endl;
		float sectorDirLocalMap = (i + 0.5) * localPlannerParams.histResolution - 180;
		float sectorToRobotDiff = min(fabs(sectorDirLocalMap - robotDirLocalMap), fabs(fabs(sectorDirLocalMap - robotDirLocalMap) - 360));

		float score = min(fabs(goalDirLocalMap - sectorDirLocalMap), fabs(fabs(goalDirLocalMap - sectorDirLocalMap) - 360))/180;	//angular distance to goal penalty
		score += (sectorToRobotDiff > 50) ? localPlannerParams.backwardsPenalty : 0.0f;
		score += histSectors[i];

		if(localPlannerParams.debug >= 2){
			cout << "sector " << i << ", score angular = ";
			cout <<	min(fabs(goalDirLocalMap - sectorDirLocalMap), fabs(fabs(goalDirLocalMap - sectorDirLocalMap) - 360))/180;
			cout << ", score backwards = " << ((sectorToRobotDiff > 90) ? localPlannerParams.backwardsPenalty : 0.0f);
			cout << ", score histogram = " << histSectors[i] << endl;
			cout << "score = " << score << endl;
		}

		if (bestSectorID == -1 || bestSectorScore > score) {
			bestSectorScore = score;
			bestSectorID = i;
		}
	}

	//middle of best sector
	float bestDirection = (bestSectorID + 0.5) * localPlannerParams.histResolution - 180;
	if(localPlannerParams.debug >= 1){
		cout << "FoundSector: " << bestSectorID << endl;
		cout << "BestSector score: " << bestSectorScore << endl;
	}
	return bestDirection;
}