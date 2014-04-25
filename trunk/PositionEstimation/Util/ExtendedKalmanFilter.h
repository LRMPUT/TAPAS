#include <opencv2/core/core.hpp>

class ExtendedKalmanFilter {

	cv::Mat Rgps, Rimu, Renc, Q; // Covariances of correct/predict

	cv::Mat I; // Identity
	cv::Mat K; // Kalman Gain
	float dt;
	cv::Mat x_apriori, x_aposteriori; // 4x1 matrices
	cv::Mat P_apriori, P_aposteriori; // 4x4 matrices
	cv::Mat Hgps, Himu, Henc;

public:
	ExtendedKalmanFilter(float _Q, float _Rgps, float _Rimu, float _Renc, float _dt);

	void predict(float dt);

	void correctGPS(cv::Mat gpsMeasurement);
	void correctIMU(cv::Mat imuMeasurement);
	void correctEncoder(cv::Mat encMeasurement);

private:
	// TODO: Fill jacobian with real values
	cv::Mat jacobian();
	// TODO: Check state implementation in MATLAB
	cv::Mat state();
};
