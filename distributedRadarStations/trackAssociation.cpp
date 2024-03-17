#include "trackAssociation.h"
#include "functions.h"
#include "Hungarian.h"

using namespace std;
using namespace Eigen;

void TrackAssociation::covarianceIntersection(VectorXd X1, MatrixXd P1, VectorXd X2, MatrixXd P2, pair<VectorXd, MatrixXd>& fusion)
{
	vector<Index> indices{ 0,1,3,4,6,7 };
	MatrixXd P1part = P1(indices, indices);
	MatrixXd InvP1 = P1part.inverse();
	MatrixXd P2part = P2(indices, indices);
	MatrixXd InvP2 = P2part.inverse();
	double w1 = ((InvP1 + InvP2).determinant() - InvP2.determinant() + InvP1.determinant()) / (2 * (InvP1 + InvP2).determinant());
	double w2 = 1 - w1;
	MatrixXd fusedCovariance = (InvP1 * w1 + InvP2 * w2).inverse();
	VectorXd fusedState = fusedCovariance * (w1 * InvP1 * X1(indices) + w2 * InvP2 * X2(indices));
	fusion.second(indices, indices) = fusedCovariance;
	fusion.first(indices) = fusedState;
}

void TrackAssociation::radarTrackAssociation(Parameters parameters, map<uint32_t, pair<VectorXd, MatrixXd>>& tracks1,
	map<uint32_t, pair<VectorXd, MatrixXd>>& tracks2, vector<int>& associatedIndex)
{
	double threshold = 0;
	size_t numTracks1 = tracks1.size();
	size_t numTracks2 = tracks2.size();
	MatrixXd distMatrix = MatrixXd::Zero(numTracks1, numTracks2);
	vector<Index> indices{ 0,1,3,4,6,7 };
	double minElement = 0;
	if (parameters.getDistanceMeasure().compare("Mahalanobis") == 0)
	{
		threshold = parameters.thresholdMahalanobis();
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator track1 = tracks1.begin(); track1 != tracks1.end(); track1++)
		{
			Index row = distance(tracks1.begin(), track1);
			VectorXd X1 = track1->second.first(indices);
			MatrixXd P1 = track1->second.second(indices, indices);
			//cout << X1.transpose() << endl;
			//cout << "P1 = " << P1 << endl;
			for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator track2 = tracks2.begin(); track2 != tracks2.end(); track2++)
			{
				Index column = distance(tracks2.begin(), track2);
				VectorXd X2 = track2->second.first(indices);
				MatrixXd P2 = track2->second.second(indices, indices);
				//cout << X2.transpose() << endl;
				//cout << "P2 = " << P2 << endl;
				distMatrix(row, column) = pow((X1 - X2).transpose() * ((P1 + P2) / 2).inverse() * (X1 - X2), 0.5);
				//cout << "dist = " << distMatrix(row, column) << endl;
			}
		}
	}
	else if (parameters.getDistanceMeasure().compare("Eculidean") == 0)
	{
		threshold = parameters.thresholdEuclidean();
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator track1 = tracks1.begin(); track1 != tracks1.end(); track1++)
		{
			Index row = distance(tracks1.begin(), track1);
			VectorXd X1 = track1->second.first(indices);
			//cout << "fused tarck state = " << X1.transpose() << endl;
			for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator track2 = tracks2.begin(); track2 != tracks2.end(); track2++)
			{
				Index column = distance(tracks2.begin(), track2);
				VectorXd X2 = track2->second.first(indices);
				//cout << "node tarck state = " << X2.transpose() << endl;
				distMatrix(row, column) = (X1 - X2).norm();
			}
		}
	}
	//cout << distMatrix << endl;
	minElement = distMatrix.minCoeff();
	distMatrix = distMatrix.array() - minElement;
	vector<vector<double>> costMatrix(distMatrix.rows());
	Functions::matrixXd2Vector(distMatrix, costMatrix);
	HungarianAlgorithm HungAlgo;
	vector<int> assignment;
	double cost = HungAlgo.Solve(costMatrix, assignment);
	for (Index i = 0; i < distMatrix.rows(); i++)
		if (assignment[i] != -1)
			if (distMatrix(i, assignment[i]) < threshold - minElement)
				associatedIndex[i] = assignment[i];
}