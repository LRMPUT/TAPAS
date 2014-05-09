#include <opencv2/core/core.hpp>
#include <cmath>

class ExtendedKalmanFilter {

	cv::Mat Rgps, Rimu, Renc, Q; // Covariances of correct/predict

	cv::Mat I; // Identity
	float dt;
	cv::Mat x_apriori, x_aposteriori; // 4x1 matrices
	cv::Mat P_apriori, P_aposteriori; // 4x4 matrices
	cv::Mat Hgps, Himu, Henc;

public:
	ExtendedKalmanFilter(float _Q = 10, float _Rgps = 20, float _Rimu = 2000/180*M_PI, float _Renc = 0.01, float _dt = 0.01);

	void predict(float dt);

	cv::Mat correctGPS(cv::Mat gpsMeasurement);
	cv::Mat correctIMU(cv::Mat imuMeasurement);
	cv::Mat correctEncoder(cv::Mat encMeasurement);

private:
	// TODO: Fill jacobian with real values
	cv::Mat jacobian();
	// TODO: Check state implementation in MATLAB
	cv::Mat state();
};
