#pragma once
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "receivingData.h"
#include "parameters.h"
#include <set>

class Track
{
private:
	bool m_survival;
	const uint32_t m_id;
	const double m_birthTime;
	double m_lastUpdateTime;
	uint32_t m_numUpdates;
	std::vector<Eigen::VectorXd> m_state;
	std::vector<Eigen::MatrixXd> m_covariance;
	std::vector<uint32_t> m_type;
	std::vector<double> m_timeStamp; //write data when prediction only or update
	std::vector<Eigen::VectorXd> m_velocity;
	std::vector<double> m_height;
public:
	Track();
	//~Track();
	Track(uint32_t id, double birthTime);
	bool survive() const;
	const uint32_t id() const;
	const double birthTime() const;
	double getLatestTimeStamp() const;
	void trackInitialization(Eigen::VectorXd state, Eigen::MatrixXd covariance, double timeStamp);
	void trackInitialization(Eigen::VectorXd state, Eigen::MatrixXd covariance, double timeStamp, Eigen::VectorXd velocity,
		int type, double height);
	void statePrediction(double currentTimeStamp, double sigma);
	void stateUpdate(Eigen::MatrixXd H, Eigen::MatrixXd R, Eigen::VectorXd position, Eigen::VectorXd velocity, 
		double height, uint32_t type, double timeStamp);
	void stateUpdate(Eigen::MatrixXd H, Eigen::MatrixXd R, Eigen::VectorXd position, double timeStamp);
	void writeSrurviveStatus(bool survive);
	void writeTimeStamp(double timeStamp);
	void getState(std::vector<Eigen::VectorXd>& states) const;
	std::pair<Eigen::VectorXd, Eigen::MatrixXd> extrapolateTrack(double referenceTime, double sigma);
	void writePredictedInfo(Eigen::VectorXd state, Eigen::MatrixXd covariance, double timeStamp);
	double getLastUpdateTime() const;
	void replaceLastState(Eigen::VectorXd state, Eigen::MatrixXd covariance);
	std::pair<Eigen::VectorXd, Eigen::MatrixXd> getCurrentState();
};

class RadarTrack
{
private:
	std::vector<uint32_t> m_trackList; // survived track ids
	std::vector<size_t> m_survivedTrackIndex; //survived track indices in m_track
	std::vector<Track> m_track;
public:
	RadarTrack();
	//~RadarTrack();
	//friend class HierarchicalRadarFusion;
	void radarTrackFiltering(Parameters& parameters, ReceivedRadarTrack& data); //only filtering with received data
	void getTracks(std::map<uint32_t, std::vector<Eigen::VectorXd>>& tracks);
	void getSurvivedTrackIndex(std::vector<size_t>& survivedTrackIndex) const;
	uint32_t getTrackID(size_t index) const;
	std::pair<Eigen::VectorXd, Eigen::MatrixXd> extrapolateTrack(Parameters& parameters, double fusionTime, size_t index);
};

class FusedTrack
{
private:
	std::map<uint32_t, Track> m_track; //current survived fused tracks
	//step->fused track id->associated track id->radar/group id (all steps)
	std::map<double, std::map<uint32_t, std::map<uint32_t, uint32_t>>> m_associations; 
	std::map<uint32_t, Track> m_terminatedTrack; //terminated fused tracks
	// number of associations with sub-hierarchical tracks, radar/group id-> track id->fusedtrack id ->number of associations
	std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, uint32_t>>> m_numAssociations; //rbegin() get maximum
	// table contains indices of sub-hierarchical tracks associated with the fused tracks
	std::map<uint32_t, std::map<uint32_t, uint32_t>> m_associationTable; // fused track id->track id->group/radar id
	//fusedtrack id->group id->node track id->number of associations
	std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, uint32_t>>> m_associationHistory;
	//radar/group id-> association confirmed node ids
	std::map<uint32_t, std::set<uint32_t>> m_associationConfirmedTrackID;
	//fused tarck id->node track id->group/radar id
	std::map<uint32_t, std::map<uint32_t, uint32_t>> m_sources; 
public:
	FusedTrack();
	FusedTrack(std::vector<uint32_t>& ids);
	//~FusedTrack();
	size_t getNumTracks();
	size_t getNumTerminatedTracks();
	void initializeNewFusedTrack(uint32_t id, double TimeStamp, Eigen::VectorXd state, Eigen::MatrixXd covariance);
	//friend class HierarchicalRadarFusion;
	bool findNodeTrackIDInAssociationTable(uint32_t fusedTrackID, uint32_t nodeTrackID);
	void getInfoFromAssociationTable(Parameters& parameters, double fusionTime, std::map<uint32_t, RadarTrack>& radarTracks, 
		std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations,
		std::map<uint32_t, std::vector<size_t>>& associatedNodeTrackIndices,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void getInfoFromAssociationTable(Parameters& parameters, double fusionTime, std::map<uint32_t, FusedTrack>& nodeTracks,
		std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations,
		std::map<uint32_t, std::vector<uint32_t>>& associatedNodeTrackIDs,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void getInfoFromAssociationTable(Parameters& parameters, double fusionTime, 
		std::map<uint32_t, FusedTrack>& nodeTracks, std::vector<uint32_t> subgroupIDs,
		std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations,
		std::map<uint32_t, std::vector<uint32_t>>& associatedNodeTrackIDs,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void updateTrackFromAssociationTable(Parameters& parameters, double fusionTIme,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void statePrediction(double fusionTime, double sigma, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& states);
	void stateUpdate(Parameters& parameters, double fusionTime, std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& states);
	void initializeAssociatedState(std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void writeAssociationInfo(uint32_t groupID, uint32_t nodeTrackID, uint32_t fusedTrackID);
	void writeAssociation(uint32_t groupID, uint32_t nodeTrackID, uint32_t fusedTrackID, std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations);
	void writeAssociation(std::pair<double, std::map<uint32_t, std::map<uint32_t, uint32_t>>> associations);
	void updateAssociationTable(uint32_t threshold);
	void getAssociatedTrackID(uint32_t id, std::pair<bool, uint32_t>& association);
	void getAssociationHistory(std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, uint32_t>>>& history);
	void writeSources(std::pair<uint32_t, std::map<uint32_t, uint32_t>>& sources);
	void getSurvivedTrack(std::map<uint32_t, Track>& tracks);
	void getTerminatedTrack(std::map<uint32_t, Track>& tracks);
	std::vector<uint32_t> getUnassociatedTrackID(std::vector<uint32_t> associatedTrackIDs);
	std::pair<Eigen::VectorXd, Eigen::MatrixXd> getLatestTrackState(uint32_t trackID);
	std::map<uint32_t, std::vector<Eigen::VectorXd>> getTracks();
};

class HierarchicalRadarFusion
{
private:
	uint32_t m_numGroups; // set in assignRadarGroup(), number of radar groups
	uint32_t m_numHierarchies;
	std::vector<uint32_t> m_radarID;
	std::map<uint32_t, uint32_t> m_radarGroup; //pair<radarID, groupIndex>
	std::queue<std::map<uint32_t, std::map<uint32_t, ReceivedRadarTrack>>> m_receivedData; //time->group->radarID->track 
	std::map<uint32_t, std::map<uint32_t, RadarTrack>> m_radarTrack; //group->radarID->track   filtered results
	// fused tracks in each hierarchy (without the toppest one, i.e., system track) hierarchy index->group index->fused track (each hierarchy has more rhan one group)
	// e.g. 32 radars with each group 4 radars -> 2 hierarchies(first hierarchy 8 groups, second hierarchy 2 groups)   
	std::map<uint32_t, std::map<uint32_t, FusedTrack>> m_hierarchicalFusedTrack;
	FusedTrack m_systemTrack;
	std::map<uint32_t, std::map<uint32_t, std::vector<uint32_t>>> m_subheirarchyIDs;
public:
	HierarchicalRadarFusion();
	~HierarchicalRadarFusion();
	uint32_t numGroups() const;
	// read all radar id from fileName and store them in m_radarID
	void getRadarID(std::string fileName); 
	// divide radars to groups, each group with numRadarsOfaGroup radars, store the groups in m_radarGroup
	void assignRadarGroup(const uint32_t numRadarsOfaGroup, uint32_t groupIDFactor); //at least 2 radars in each group
	// write received radar into m_receivedData for each group with respect to time stamp
	void wrapReceivedData(std::queue<std::map<uint32_t, ReceivedRadarTrack>>& data);
	// filtering radar track to get covariance for fusion, the filterd tracks are stored in m_radarTrack
	void initializeRadarTrackInfo(); //initialize m_radarTrack
	void parallelRadarTrackFiltering(Parameters parameters, std::pair<uint32_t, std::map<uint32_t, ReceivedRadarTrack>> wrappedData); //parallel filtering
	void RadarTrackFiltering(Parameters& parameters, ReceivedData& receivedData); // filtering interface
	// hierarchical fusion
	void initializeHierarchicalFusedTrack(const uint32_t numMembers, uint32_t hierarchyIDFactor, uint32_t groupIDFactor); //number of members in each hierachy, i.e., Parameters::m_numRadarsOfaGroup
	void getRadarTrackInfo(Parameters& parameters, double fusionTime, bool useSystemTrack,
		std::map<uint32_t, FusedTrack>::iterator hierarchicalFusedTrack,
		std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations, 
		std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& fusedTrackStates,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& nodeTrackStates,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void parallelRadarTrackFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy,
		std::map<uint32_t, FusedTrack>::iterator hierarchicalFusedTrack, bool useSystemTrack);
	void groupTrackFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy, uint32_t group,
		FusedTrack* fusedTracks, std::map<uint32_t, std::map<uint32_t, uint32_t>>& associations,
		std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& fusedTrackStates,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& nodeTrackStates,
		std::map<uint32_t, std::map<uint32_t, std::pair<Eigen::VectorXd, Eigen::MatrixXd>>>& associatedStates);
	void rootHierarchyFusion(Parameters& parameters, double fusionTime, bool useSystemTrack); //lowest hierarchy, connect to original radar tracks
	void middleHierarchyFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy,
		std::map<uint32_t, FusedTrack>::iterator currrentHierarchyTrack, 
		std::map<uint32_t, FusedTrack>& lastHierarchyTrack, std::vector<uint32_t> subgroupIDs);
	void topHierarchyFusion(Parameters& parameters, double fusionTime, std::map<uint32_t, FusedTrack>& nodeTrack);
	void hierarchicalFusion(Parameters& parameters, double fusionTime);
	void getAssociationHistory(std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, uint32_t>>>>& history);
	void displayAssociationHistory(std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, std::map<uint32_t, uint32_t>>>>& history);
	// plot results (tracks)
	void plotFilterdRadarTrack(std::string coordinate, std::string plotOrWrite);
	void plotSystemTrack(std::string coordinate, std::string plotOrWrite);
};