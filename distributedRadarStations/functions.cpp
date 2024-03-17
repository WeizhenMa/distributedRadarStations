#include "functions.h"
#include <algorithm>
#include <chrono>
#include <random>

using namespace std;
using namespace Eigen;

pair<bool, size_t> Functions::findIndex(int index, vector<uint32_t> indices)
{
	pair<bool, size_t> found(false, 0);
	if (!indices.empty())
	{
		vector<uint32_t>::iterator iter;
		iter = find(indices.begin(), indices.end(), index);
		if (iter != indices.end())
		{
			found.first = true;
			found.second = distance(indices.begin(), iter);
		}
	}
	return found;
}

bool Functions::isMember(uint32_t index, vector<uint32_t> indices)
{
	bool member = false;
	if (!indices.empty())
	{
		vector<uint32_t>::iterator iter = find(indices.begin(), indices.end(), index);
		if (iter != indices.end()) member = true;
	}
	return member;
}

bool Functions::isMember(size_t index, vector<size_t> indices)
{
	bool member = false;
	if (!indices.empty())
	{
		vector<size_t>::iterator iter = find(indices.begin(), indices.end(), index);
		if (iter != indices.end()) member = true;
	}
	return member;
}

double Functions::uniformRand(double lower, double upper)
{
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine gen(seed);
	uniform_real_distribution<> uniform(lower, upper);
	return uniform(gen);
}

double Functions::uniformRand()
{
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine gen(seed);
	uniform_real_distribution<> uniform(0.00001, 0.99999);
	return uniform(gen);
}

void Functions::matrixXd2Vector(MatrixXd original, vector<vector<double>>& reformed)
{
	for (Index i = 0; i < original.rows(); i++)
		for (Index j = 0; j < original.cols(); j++) 
			reformed[i].emplace_back(original(i, j));
}

bool Functions::isContaining(uint32_t id, vector<set<uint32_t>> ids)
{
	bool containing = false;
	for (set<uint32_t>& s : ids)
	{
		set<uint32_t>::iterator iter = s.find(id);
		if (iter != s.end())
		{
			containing = true;
			break;
		}
	}
	return containing;
}