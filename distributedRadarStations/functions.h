#pragma once
#include <map>
#include <vector>
#include <Eigen/Dense>
#include <set>

class Functions
{
public:
	static std::pair<bool, size_t> findIndex(int index, std::vector<uint32_t> indices);
	static bool isMember(uint32_t index, std::vector<uint32_t> indices);
	static bool isMember(size_t index, std::vector<size_t> indices);
	static double uniformRand(double lower, double upper);
	static double uniformRand();
	static void matrixXd2Vector(Eigen::MatrixXd original, std::vector<std::vector<double>>& reformed);
	static bool isContaining(uint32_t id, std::vector<std::set<uint32_t>> ids);
};