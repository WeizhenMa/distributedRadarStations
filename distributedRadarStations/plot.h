#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>

class OpencvPlot
{
public:
	static void plotTracks(std::map<uint32_t, std::vector<Eigen::VectorXd>> states, std::string coordinate);
};

class MatPlot
{
public:
	static void plotTracks(std::map<uint32_t, std::vector<Eigen::VectorXd>> states, std::string coordinate);
};
