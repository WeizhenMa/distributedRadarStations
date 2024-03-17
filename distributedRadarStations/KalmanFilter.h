#pragma once
#include <map>
#include <Eigen/Dense>

class KalmanFilter
{
public:
	static std::pair<Eigen::VectorXd, Eigen::MatrixXd> predict(Eigen::VectorXd X, Eigen::MatrixXd P, 
		Eigen::MatrixXd F, Eigen::MatrixXd Q);
	static std::pair<Eigen::VectorXd, Eigen::MatrixXd> update(Eigen::VectorXd X, Eigen::MatrixXd P, 
		Eigen::MatrixXd H, Eigen::MatrixXd R, Eigen::VectorXd Z);
};