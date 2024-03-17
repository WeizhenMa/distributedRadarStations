#include "KalmanFilter.h"
#include <iostream>

using namespace std;
using namespace Eigen;

pair<VectorXd, MatrixXd> KalmanFilter::predict(VectorXd priorState, MatrixXd priorCovariance, MatrixXd F, MatrixXd Q)
{
	VectorXd predictedState = F * priorState;
	MatrixXd predictedCovariance = F * priorCovariance * F.transpose() + Q;
	return pair<VectorXd, MatrixXd>(predictedState, predictedCovariance);
}

pair<VectorXd, MatrixXd> KalmanFilter::update(VectorXd priorState, MatrixXd priorCovariance, MatrixXd H, MatrixXd R, VectorXd Z)
{
	//cout << "measurement = " << Z.transpose() << endl;
	//cout << "prior state = " << priorState.transpose() << endl;
	VectorXd predictedMeasurement = H * priorState;
	//cout << "predicted measurement = " << predictedMeasurement.transpose() << endl;
	MatrixXd innovationCovariance = H * priorCovariance * H.transpose() + R;
	MatrixXd K = priorCovariance * H.transpose() * innovationCovariance.inverse();
	//cout << "delta = " << (Z - predictedMeasurement).norm() << endl;
	VectorXd posteriorState = priorState + K * (Z - predictedMeasurement);
	MatrixXd posteriorCovariance = (MatrixXd::Identity(priorCovariance.rows(), priorCovariance.rows()) - K * H) * priorCovariance;
	//cout << "posterior state = " << posteriorState.transpose() << endl;
	//cout << "delta = " << (posteriorState({ 0,3,6 }) - Z.head(3)).norm() << endl;
	return pair<VectorXd, MatrixXd>(posteriorState, posteriorCovariance);
}