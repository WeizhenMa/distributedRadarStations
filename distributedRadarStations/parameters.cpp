#include "parameters.h"

using namespace std;
using namespace Eigen;

Parameters::Parameters(const uint32_t numRadarsOfaGroup, const double fusionInterval)
	: m_numRadarsOfaGroup(numRadarsOfaGroup), m_fusionInterval(fusionInterval)
{
	//this->m_numRadarsOfaGroup = 1;
	this->m_smallProcessNoiseStd = 10;
	this->m_largeProcessNoiseStd = 20;
	this->m_measurementNoiseStd = 100;
	int model = 2;
	MatrixXd R;
	MatrixXd H;
	vector<Index> indices;
	vector<Index> indices1({ 0,3,6 });
	vector<Index> indices2({ 0, 1, 3, 4, 6 });
	switch (model)
	{
	case 1:
		R = pow(this->m_measurementNoiseStd, 2) * MatrixXd::Identity(3, 3);
		H = MatrixXd::Zero(3, 9);
		H(0, 0) = 1; H(1, 3) = 1; H(2, 6) = 1;
		indices.assign(indices1.begin(), indices1.end());
		break;
	case 2:
		R = MatrixXd::Zero(5, 5);
		R.topLeftCorner(3,3) = pow(this->m_measurementNoiseStd, 2) * MatrixXd::Identity(3, 3);
		R.bottomRightCorner(2, 2) = pow(5, 2) * MatrixXd::Identity(5, 5);
		H = MatrixXd::Zero(5, 9);
		H(0, 0) = 1; H(1, 3) = 1; H(2, 6) = 1;
		H(3, 1) = 1; H(4, 4) = 1;
		indices.assign(indices2.begin(), indices2.end());
		break;
	}
	this->m_R = R;
	this->m_H = H;
	this->m_indices.assign(indices.begin(), indices.end());
	MatrixXd P = MatrixXd::Zero(9, 9);
	double sigmaPositionP = 100, sigmaVelocityP = 50, sigmaAccelerationP = 10;
	Matrix3d Pblock = Matrix3d::Zero(3, 3);
	Pblock(0, 0) = pow(sigmaPositionP, 2);
	Pblock(1, 1) = pow(sigmaVelocityP, 2);
	Pblock(2, 2) = pow(sigmaAccelerationP, 2);
	P({ 0,1,2 }, { 0,1,2 }) = Pblock;
	P({ 3,4,5 }, { 3,4,5 }) = Pblock;
	P({ 6,7,8 }, { 6,7,8 }) = Pblock;
	this->m_initialStateCovariance = P;
	this->m_nodeTrackTerminateTime = 20;
	this->m_fusedTrackTerminateTime = 50;
	this->m_thresholdEuclidean = 8000; //5000 too small, some tracks are unassociated
	this->m_thresholdMahalanobis = 50;
	this->m_invalidAssignment = -1;
	this->m_thresholdConfirmedAssociation = 20;
	this->m_distanceMeasure = "Eculidean";
	this->m_trackIDFactor = 1e4;
	this->m_hierarchyIDFactor = 1e4;
	this->m_groupIDFactor = 1e3;
}

const uint32_t Parameters::numRadarsOfaGroup() const
{
	return this->m_numRadarsOfaGroup;
}

const double Parameters::fusionInterval() const
{
	return this->m_fusionInterval;
}

const uint32_t Parameters::trackIDFactor() const
{
	return this->m_trackIDFactor;
}

double Parameters::smallProcessNoiseStd() const
{
	return this->m_smallProcessNoiseStd;
}

double Parameters::largeProcessNoiseStd() const
{
	return this->m_largeProcessNoiseStd;
}

double Parameters::measurementNoiseStd() const
{
	return this->m_measurementNoiseStd;
}

MatrixXd Parameters::R() const
{
	return this->m_R;
}

MatrixXd Parameters::H() const
{
	return this->m_H;
}

MatrixXd Parameters::initialStateCovariance() const
{
	return this->m_initialStateCovariance;
}

double Parameters::nodeTrackTerminateTime() const
{
	return this->m_nodeTrackTerminateTime;
}

double Parameters::fusedTrackTerminateTime() const
{
	return this->m_fusedTrackTerminateTime;
}

double Parameters::thresholdMahalanobis() const
{
	return this->m_thresholdMahalanobis;
}
double Parameters::thresholdEuclidean() const
{
	return this->m_thresholdEuclidean;
}

int Parameters::invalidAssignment() const
{
	return this->m_invalidAssignment;
}

uint32_t Parameters::thresholdConfirmedAssociation() const
{
	return this->m_thresholdConfirmedAssociation;
}

vector<Index> Parameters::getIndices() const
{
	return this->m_indices;
}

void Parameters::setDistanceMeasure(string measure)
{
	this->m_distanceMeasure = measure;
}

string Parameters::getDistanceMeasure() const
{
	return this->m_distanceMeasure;
}

uint32_t Parameters::getHierarchyIDFactor() const
{
	return this->m_hierarchyIDFactor;
}

uint32_t Parameters::getGroupIDFactor() const
{
	return this->m_groupIDFactor;
}

MatrixXd ConstantVelocity::Fp(double dt)
{
	MatrixXd F = MatrixXd::Zero(9, 9);
	MatrixXd CVFBlock = MatrixXd::Zero(3, 3);
	CVFBlock(0, 0) = 1; CVFBlock(0, 1) = dt; CVFBlock(1, 1) = 1;
	F({ 0,1,2 }, { 0,1,2 }) = CVFBlock;
	F({ 3,4,5 }, { 3,4,5 }) = CVFBlock;
	F({ 6,7,8 }, { 6,7,8 }) = CVFBlock;
	/*MatrixXd CAFBlock = MatrixXd::Zero(3, 3);
	CAFBlock(0, 0) = 1; CAFBlock(0, 1) = dt; CAFBlock(0, 2) = pow(dt, 2) / 2;
	CAFBlock(1, 1) = 1; CAFBlock(1, 2) = dt; CAFBlock(2, 2) = 1;
	F({ 0,1,2 }, { 0,1,2 }) = CAFBlock;
	F({ 3,4,5 }, { 3,4,5 }) = CAFBlock;
	F({ 6,7,8 }, { 6,7,8 }) = CAFBlock;*/
	return F;
}

MatrixXd ConstantVelocity::Fn(double dt)
{
	MatrixXd F = MatrixXd::Zero(9, 9);
	MatrixXd CVFBlock = MatrixXd::Zero(3, 3);
	CVFBlock(0, 0) = 1; CVFBlock(0, 1) = -dt; CVFBlock(1, 1) = 1;
	F({ 0,1,2 }, { 0,1,2 }) = CVFBlock;
	F({ 3,4,5 }, { 3,4,5 }) = CVFBlock;
	F({ 6,7,8 }, { 6,7,8 }) = CVFBlock;
	return F;
}

MatrixXd ConstantVelocity::Q(double dt, double sigma)
{
	MatrixXd G = MatrixXd::Zero(9, 3);
	G(0, 0) = pow(dt, 2) / 2; G(1, 0) = dt; G(2, 0) = 0;
	G(3, 1) = pow(dt, 2) / 2; G(4, 1) = dt; G(5, 1) = 0;
	G(6, 2) = pow(dt, 2) / 2; G(7, 2) = dt; G(8, 2) = 0;
	G *= sigma;
	MatrixXd Q = G * G.transpose();
	return Q;
}