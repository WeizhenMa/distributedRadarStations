#pragma once
#include <map>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "parameters.h"

class TrackAssociation
{
public:
	static void covarianceIntersection(Eigen::VectorXd X1, Eigen::MatrixXd P1, Eigen::VectorXd X2, Eigen::MatrixXd P2, std::pair<Eigen::VectorXd, Eigen::MatrixXd>& fusion);
	static void radarTrackAssociation(Parameters parameters, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& tracks1,
		std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& tracks2, std::vector<int>& associatedIndex);
};