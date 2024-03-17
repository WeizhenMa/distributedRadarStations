#include <iostream>
#include "receivingData.h"
#include "hierarchicalRadarFusion.h"
#include <thread>
#include <numeric>


using namespace std;
using namespace Eigen;

// 6 targets
// 32 radars

//when use iterator, think twice does it matter if it the container is empty, see FusedTrack::updateAssociationTable

int main()
{
	/*map<int, int> m;
	m.emplace(1, 1);
	m.emplace(2, 2);
	m.emplace(3, 3);
	map<int, int>::iterator iter = m.begin();
	for (; iter != m.end(); iter++)
		cout << iter->second << endl;
	advance(iter, -1);
	map<int, int>::iterator iter1 = prev(iter, 1);*/
	double startTime = 1000;
	double stopTime = 2700;
	const uint32_t numRadarsOfaGroup = 4; //must greater than 1 and smaller than the number of radars
	const double fusionInterval = 0.5; //seconds
	Parameters parameters(numRadarsOfaGroup, fusionInterval);
	parameters.setDistanceMeasure("Eculidean");
	string jsonDataFileFolder = "..\\sendingData\\jsonData\\";
	NewtworkConfiguration network;
	ReceivedData receivedData;
	receivedData.writeTimeStamps("timeStamps.txt");
	HierarchicalRadarFusion hierRadarFusion;
	hierRadarFusion.getRadarID("radarID.txt");
	hierRadarFusion.assignRadarGroup(parameters.numRadarsOfaGroup(), parameters.getGroupIDFactor());
	hierRadarFusion.initializeRadarTrackInfo();
	hierRadarFusion.initializeHierarchicalFusedTrack(parameters.numRadarsOfaGroup(), parameters.getHierarchyIDFactor(), parameters.getGroupIDFactor());
	pair<int, int> stepIndices(0, 0);
	clock_t start = clock();
	for (double fusionTime = startTime + fusionInterval; fusionTime < stopTime; fusionTime += fusionInterval)
	{
		clock_t singleStepStart = clock();
		cout << "******************fusion time = " << fusionTime;
		receivedData.getDataIndices(fusionTime, stepIndices);
		receivedData.writeDataFromJson(jsonDataFileFolder, fusionTime, stepIndices, parameters.trackIDFactor());
		hierRadarFusion.RadarTrackFiltering(parameters, receivedData);
		hierRadarFusion.hierarchicalFusion(parameters, fusionTime);
		//hierRadarFusion.rootHierarchyFusion(parameters, fusionTime, false);
		clock_t singleStepStop = clock();
		cout << ", single step running time = " << ((double)singleStepStop - (double)singleStepStart) / CLOCKS_PER_SEC << endl;
	}
	clock_t stop = clock();
	double runningTime = ((double)stop - (double)start) / CLOCKS_PER_SEC;
	cout << "running time = " << runningTime << endl;
	map<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>> associationHistory;
	hierRadarFusion.getAssociationHistory(associationHistory);
	hierRadarFusion.displayAssociationHistory(associationHistory);
	hierRadarFusion.plotSystemTrack("lla", "plot");
	//hierRadarFusion.plotFilterdRadarTrack("lla", "plot");
	//thread receivingDataThread(&ReceivedData::receivingData, ref(receivedData), network);
	//receivingDataThread.join();
	system("pause");
	return 0;
}