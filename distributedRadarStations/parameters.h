#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

class Parameters
{
private:
	const uint32_t m_numRadarsOfaGroup; //number of radars in each group, must greater than 1
	const double m_fusionInterval;
	uint32_t m_trackIDFactor;
	double m_smallProcessNoiseStd;
	double m_largeProcessNoiseStd;
	double m_measurementNoiseStd;
	Eigen::MatrixXd m_R;
	Eigen::MatrixXd m_H;
	Eigen::MatrixXd m_initialStateCovariance;
	double m_nodeTrackTerminateTime; //in seconds
	double m_fusedTrackTerminateTime; //in seconds
	// track association
	double m_thresholdMahalanobis;
	double m_thresholdEuclidean;
	int m_invalidAssignment;
	uint32_t m_thresholdConfirmedAssociation;
	std::vector<Eigen::Index> m_indices;
	std::string m_distanceMeasure; //track association distance measure : Euclidean or Mahalanobis distance
	uint32_t m_hierarchyIDFactor;
	uint32_t m_groupIDFactor;
public:
	Parameters(const uint32_t numRadarsOfaGroup, const double fusionInterval);
	const uint32_t numRadarsOfaGroup() const;
	const double fusionInterval() const;
	const uint32_t trackIDFactor() const;
	double smallProcessNoiseStd() const;
	double largeProcessNoiseStd() const;
	double measurementNoiseStd() const;
	Eigen::MatrixXd R() const;
	Eigen::MatrixXd H() const;
	Eigen::MatrixXd initialStateCovariance() const;
	double nodeTrackTerminateTime() const;
	double fusedTrackTerminateTime() const;
	double thresholdMahalanobis() const;
	double thresholdEuclidean() const;
	int invalidAssignment() const;
	uint32_t thresholdConfirmedAssociation() const;
	std::vector<Eigen::Index> getIndices() const;
	void setDistanceMeasure(std::string measure);
	std::string getDistanceMeasure() const;
	uint32_t getHierarchyIDFactor() const;
	uint32_t getGroupIDFactor() const;
};

class ConstantVelocity
{
public:
	static Eigen::MatrixXd Fp(double dt);
	static Eigen::MatrixXd Fn(double dt);
	static Eigen::MatrixXd Q(double dt, double sigma);
};