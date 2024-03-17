#include "hierarchicalRadarFusion.h"
#include <fstream>
#include <iostream>
#include "functions.h"
#include "KalmanFilter.h"
#include <thread>
#include "plot.h"
#include "coordinateTransition.h"
#include "trackAssociation.h"
#include <algorithm>

#ifndef D2R
#define D2R 0.017453292519943
#endif 

#ifndef wgs84
#define wgs84 WGS84()
#endif 

using namespace std;
using namespace Eigen;

Track::Track() :m_id(0), m_birthTime(0) 
{
	this->m_survival = true;
	this->m_lastUpdateTime = 0;
	this->m_numUpdates = 0;
};

//Track::~Track() {};

Track::Track(uint32_t id, double birthTime) :m_id(id), m_birthTime(birthTime)
{
	this->m_survival = true;
	this->m_lastUpdateTime = 0;
	this->m_numUpdates = 0;
}

bool Track::survive() const
{
	return this->m_survival;
}

const uint32_t Track::id() const
{
	return this->m_id;
}

const double Track::birthTime() const
{
	return this->m_birthTime;
}

double Track::getLatestTimeStamp() const
{
	return this->m_timeStamp.back();
}

void Track::statePrediction(double currentTimeStamp, double sigma)
{
	double dt = currentTimeStamp - this->m_timeStamp.back();
	pair<VectorXd, MatrixXd> predict = KalmanFilter::predict(this->m_state.back(), this->m_covariance.back(), 
		ConstantVelocity::Fp(dt), ConstantVelocity::Q(dt, sigma));
	this->m_state.emplace_back(predict.first);
	this->m_covariance.emplace_back(predict.second);
}

void Track::stateUpdate(MatrixXd H, MatrixXd R, VectorXd position, VectorXd velocity, double height, uint32_t type, double timeStamp)
{
	VectorXd meas = VectorXd::Zero(5);
	meas.head(3) = position;
	meas.tail(2) = velocity.head(2);
	pair<VectorXd, MatrixXd> update = KalmanFilter::update(this->m_state.back(), this->m_covariance.back(), H, R, meas);
	//pair<VectorXd, MatrixXd> update = KalmanFilter::update(this->m_state.back(), this->m_covariance.back(), H, R, position);
	this->m_state.back() = update.first;
	this->m_covariance.back() = update.second;
	this->m_velocity.emplace_back(velocity);
	this->m_height.emplace_back(height);
	this->m_type.emplace_back(type);
	this->m_timeStamp.emplace_back(timeStamp);
	this->m_lastUpdateTime = timeStamp;
}

void Track::stateUpdate(MatrixXd H, MatrixXd R, VectorXd position, double timeStamp)
{
	pair<VectorXd, MatrixXd> update = KalmanFilter::update(this->m_state.back(), this->m_covariance.back(), H, R, position);
	this->m_state.back() = update.first;
	this->m_covariance.back() = update.second;
	this->m_timeStamp.emplace_back(timeStamp);
	this->m_lastUpdateTime = timeStamp;
}

void Track::writeSrurviveStatus(bool survive)
{
	this->m_survival = survive;
}

void Track::writeTimeStamp(double timeStamp)
{
	this->m_timeStamp.emplace_back(timeStamp);
}

void Track::trackInitialization(VectorXd state, MatrixXd covariance, double timeStamp)
{
	this->m_state.emplace_back(state);
	this->m_covariance.emplace_back(covariance);
	this->m_timeStamp.emplace_back(timeStamp);
	this->m_lastUpdateTime = timeStamp;
}

void Track::trackInitialization(VectorXd state, MatrixXd covariance, double timeStamp, VectorXd velocity, int type, double height)
{
	this->m_state.emplace_back(state);
	this->m_covariance.emplace_back(covariance);
	this->m_timeStamp.emplace_back(timeStamp);
	this->m_lastUpdateTime = timeStamp;
	this->m_velocity.emplace_back(velocity);
	this->m_height.emplace_back(height);
	this->m_type.emplace_back(type);
}

void Track::getState(vector<VectorXd>& states)  const
{
	states.assign(this->m_state.begin(), this->m_state.end());
}

pair<VectorXd, MatrixXd> Track::extrapolateTrack(double referenceTime, double sigma)
{
	double dt = referenceTime - this->m_timeStamp.back();
	pair<VectorXd, MatrixXd> predict = KalmanFilter::predict(this->m_state.back(), this->m_covariance.back(),
		ConstantVelocity::Fp(dt), ConstantVelocity::Q(dt, sigma));
	return pair<VectorXd, MatrixXd>(predict.first, predict.second);
}

void Track::writePredictedInfo(VectorXd state, MatrixXd covariance, double timeStamp)
{
	this->m_state.emplace_back(state);
	this->m_covariance.emplace_back(covariance);
	this->m_timeStamp.emplace_back(timeStamp);
}

double Track::getLastUpdateTime() const
{
	return this->m_lastUpdateTime;
}

void Track::replaceLastState(VectorXd state, MatrixXd covariance)
{
	this->m_state.back() = state;
	this->m_covariance.back() = covariance;
}

pair<VectorXd, MatrixXd> Track::getCurrentState()
{
	return pair<VectorXd, MatrixXd>(this->m_state.back(), this->m_covariance.back());
}

RadarTrack::RadarTrack() {};

//RadarTrack::~RadarTrack() {};

void RadarTrack::radarTrackFiltering(Parameters& parameters, ReceivedRadarTrack& data)
{
	vector<uint32_t> detectedTargetID;
	data.readDetectedID(detectedTargetID);
	MatrixXd H = parameters.H();
	MatrixXd R = parameters.R();
	if (!detectedTargetID.empty())
	{
		vector<double> timeStamps;
		data.readTimeStamps(timeStamps);
		vector<Vector3d> position;
		data.readPosition(position);
		vector<Vector3d> velocity;
		data.readVelocity(velocity);
		vector<uint32_t> type;
		data.readTargetType(type);
		vector<double> height;
		data.readHeight(height);
		vector<uint32_t> updatedTargetId;
		double referenceTimeStamp = timeStamps.front();
		for (Track& track : this->m_track)
		{
			uint32_t trackID = track.id();
			pair<bool, size_t> found = Functions::findIndex(trackID, detectedTargetID);
			if (track.survive())
				if (found.first)
				{
					updatedTargetId.emplace_back(trackID);
					size_t index = found.second;
					track.statePrediction(timeStamps[index], parameters.smallProcessNoiseStd());
					track.stateUpdate(H, R, position[index], velocity[index], height[index], type[index], timeStamps[index]);
				}
				else
				{
					if (referenceTimeStamp - track.getLastUpdateTime() > parameters.nodeTrackTerminateTime())
						track.writeSrurviveStatus(false);
					else
					{
						track.statePrediction(referenceTimeStamp, parameters.largeProcessNoiseStd());
						track.writeTimeStamp(referenceTimeStamp);
					}
				}
			else
				if (found.first)
				{
					updatedTargetId.emplace_back(trackID);
					track.writeSrurviveStatus(true);
					size_t index = found.second;
					track.statePrediction(timeStamps[index], parameters.smallProcessNoiseStd());
					track.stateUpdate(H, R, position[index], velocity[index], height[index], type[index], timeStamps[index]);
				}
		}
		vector<uint32_t> newTargetId;
		if (updatedTargetId.empty()) 
			newTargetId.assign(detectedTargetID.begin(), detectedTargetID.end());
		else
			for (uint32_t& id : detectedTargetID)
				if (!Functions::isMember(id, updatedTargetId))
					newTargetId.emplace_back(id);
		if (!newTargetId.empty())
			for (uint32_t& id : newTargetId)
			{
				pair<bool, size_t> found = Functions::findIndex(id, detectedTargetID);
				size_t index = found.second;
				VectorXd state = VectorXd::Zero(9);
				//state({ 0,3,6 }) = position[index];
				VectorXd meas = VectorXd::Zero(9);
				meas({ 0,3,6 }) = position[index];
				meas({ 1,4,7 }) = velocity[index];
				state(parameters.getIndices()) = meas(parameters.getIndices());
				Track newTrack(id, timeStamps[index]);
				newTrack.trackInitialization(state, parameters.initialStateCovariance(), timeStamps[index], velocity[index], type[index], height[index]);
				this->m_track.emplace_back(newTrack);
			}
		int numTargets = 0;
		vector<size_t> survivedTrackIndex;
		vector<uint32_t> allTargetID;
		for (size_t t = 0; t < this->m_track.size(); t++)
		{
			if (this->m_track[t].survive())
			{
				numTargets++;
				survivedTrackIndex.emplace_back(t);
			}
			allTargetID.emplace_back(this->m_track[t].id());
		}
		this->m_survivedTrackIndex.assign(survivedTrackIndex.begin(), survivedTrackIndex.end());
		this->m_trackList.assign(allTargetID.begin(), allTargetID.end());
	}
}

void RadarTrack::getTracks(map<uint32_t, vector<VectorXd>>& tracks)
{
	for (Track& track : this->m_track)
	{
		vector<VectorXd> state;
		track.getState(state);
		tracks.emplace(pair<uint32_t, vector<VectorXd>>(track.id(), state));
	}
}

void RadarTrack::getSurvivedTrackIndex(vector<size_t>& survivedTrackIndex) const
{
	survivedTrackIndex.assign(this->m_survivedTrackIndex.begin(), this->m_survivedTrackIndex.end());
}

uint32_t RadarTrack::getTrackID(size_t index) const
{
	return this->m_track[index].id();
}

pair<VectorXd, MatrixXd> RadarTrack::extrapolateTrack(Parameters& parameters, double fusionTime, size_t index)
{
	pair<VectorXd, MatrixXd> extrapolate = this->m_track[index].extrapolateTrack(fusionTime, parameters.largeProcessNoiseStd());
	return extrapolate;
}

FusedTrack::FusedTrack() {};

FusedTrack::FusedTrack(vector<uint32_t>& ids) 
{
	for (uint32_t& id : ids)
	{
		this->m_numAssociations.emplace(pair<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>(id, map<uint32_t, map<uint32_t, uint32_t>>()));
		this->m_associationConfirmedTrackID.emplace(pair<uint32_t, set<uint32_t>>(id, set<uint32_t>()));
	}
};

//FusedTrack::~FusedTrack() {};

size_t FusedTrack::getNumTracks()
{
	return this->m_track.size();
}

size_t FusedTrack::getNumTerminatedTracks()
{
	return this->m_terminatedTrack.size();
}

void FusedTrack::initializeNewFusedTrack(uint32_t id, double timeStamp, VectorXd state, MatrixXd covariance)
{
	Track track(id, timeStamp);
	track.trackInitialization(state, covariance, timeStamp);
	this->m_track.emplace(pair<uint32_t, Track>(id, track));
}

bool FusedTrack::findNodeTrackIDInAssociationTable(uint32_t fusedTrackID, uint32_t nodeTrackID)
{
	bool found = false;
	if (!this->m_associationTable.empty())
	{
		map<uint32_t, map<uint32_t, uint32_t>>::iterator findFusedTrackID = this->m_associationTable.find(fusedTrackID);
		if (findFusedTrackID != this->m_associationTable.end())
		{
			map<uint32_t, uint32_t>::iterator foundNodeTrackID = findFusedTrackID->second.find(nodeTrackID);
			if (foundNodeTrackID != findFusedTrackID->second.end()) 
				found = true;
		}
	}
	return found;
}

void FusedTrack::getInfoFromAssociationTable(Parameters& parameters, double fusionTime, map<uint32_t, RadarTrack>& radarTracks,
	map<uint32_t, map<uint32_t, uint32_t>>& associations, map<uint32_t, vector<size_t>>& associatedNodeTrackIndices,
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	for (map<uint32_t, Track>::iterator fusedTrack = this->m_track.begin(); fusedTrack != this->m_track.end(); fusedTrack++)
	{
		associatedStates.emplace(pair<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>(fusedTrack->first, map<uint32_t, pair<VectorXd, MatrixXd>>()));
		for (map<uint32_t, RadarTrack>::iterator radarTrack = radarTracks.begin(); radarTrack != radarTracks.end(); radarTrack++)
		{
			vector<size_t> survivedTrackIndex;
			radarTrack->second.getSurvivedTrackIndex(survivedTrackIndex);
			for (size_t& index : survivedTrackIndex)
			{
				uint32_t nodeTrackID = radarTrack->second.getTrackID(index);
				if (this->findNodeTrackIDInAssociationTable(fusedTrack->first, nodeTrackID))
				{
					pair<VectorXd, MatrixXd> extrapolate = radarTrack->second.extrapolateTrack(parameters, fusionTime, index);
					associatedStates[fusedTrack->first].emplace(pair<int, pair<VectorXd, MatrixXd>>(nodeTrackID, extrapolate));
					associatedNodeTrackIndices[radarTrack->first].emplace_back(index);
					associations[fusedTrack->first].emplace(pair<uint32_t, uint32_t>(nodeTrackID, radarTrack->first));
					//cout << "fused track id = " << fusedTrack->first << ", node track id = " << nodeTrackID << endl;
				}
			}
		}
	}
}

void FusedTrack::getInfoFromAssociationTable(Parameters& parameters, double fusionTime, map<uint32_t, FusedTrack>& nodeTracks,
	map<uint32_t, map<uint32_t, uint32_t>>& associations, map<uint32_t, vector<uint32_t>>& associatedNodeTrackIDs,
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	for (map<uint32_t, Track>::iterator fusedTrack = this->m_track.begin(); fusedTrack != this->m_track.end(); fusedTrack++)
	{
		associatedStates.emplace(pair<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>(fusedTrack->first, map<uint32_t, pair<VectorXd, MatrixXd>>()));
		for (map<uint32_t, FusedTrack>::iterator nodeTrack = nodeTracks.begin(); nodeTrack != nodeTracks.end(); nodeTrack++)
		{
			for (map<uint32_t, Track>::iterator track = nodeTrack->second.m_track.begin(); track != nodeTrack->second.m_track.end(); track++)
			{
				uint32_t trackID = track->first;
				if (this->findNodeTrackIDInAssociationTable(fusedTrack->first, trackID))
				{
					pair<VectorXd, MatrixXd> state = track->second.getCurrentState();
					associatedStates[fusedTrack->first].emplace(pair<int, pair<VectorXd, MatrixXd>>(trackID, state));
					associations[fusedTrack->first].emplace(pair<uint32_t, uint32_t>(trackID, nodeTrack->first));
					associatedNodeTrackIDs[nodeTrack->first].emplace_back(trackID);
					//cout << "fused track id = " << fusedTrack->first << ", node track id = " << nodeTrackID << endl;
				}
			}
			
		}
	}
}

void FusedTrack::getInfoFromAssociationTable(Parameters& parameters, double fusionTime, map<uint32_t, FusedTrack>& nodeTracks,
	vector<uint32_t> subgroupIDs, map<uint32_t, map<uint32_t, uint32_t>>& associations, 
	map<uint32_t, vector<uint32_t>>& associatedNodeTrackIDs, map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	for (map<uint32_t, Track>::iterator fusedTrack = this->m_track.begin(); fusedTrack != this->m_track.end(); fusedTrack++)
	{
		associatedStates.emplace(pair<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>(fusedTrack->first, map<uint32_t, pair<VectorXd, MatrixXd>>()));
		for (uint32_t& id : subgroupIDs)
		{
			FusedTrack nodeTrack = nodeTracks[id];
			for (map<uint32_t, Track>::iterator track = nodeTrack.m_track.begin(); track != nodeTrack.m_track.end(); track++)
			{
				uint32_t trackID = track->first;
				if (this->findNodeTrackIDInAssociationTable(fusedTrack->first, trackID))
				{
					pair<VectorXd, MatrixXd> state = track->second.getCurrentState();
					associatedStates[fusedTrack->first].emplace(pair<int, pair<VectorXd, MatrixXd>>(trackID, state));
					associations[fusedTrack->first].emplace(pair<uint32_t, uint32_t>(trackID, id));
					associatedNodeTrackIDs[id].emplace_back(trackID);
					//cout << "fused track id = " << fusedTrack->first << ", node track id = " << nodeTrackID << endl;
				}
			}
		}
	}
}

void FusedTrack::updateTrackFromAssociationTable(Parameters& parameters, double fusionTime, map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	vector<uint32_t> terminatedTrackIDs;
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
	{
		map<uint32_t, pair<VectorXd, MatrixXd>>& states = associatedStates[track->first];
		if (states.empty()) //no associated track according to association table
		{
			if (fusionTime - track->second.getLastUpdateTime() >= parameters.fusedTrackTerminateTime()) //terminate
				terminatedTrackIDs.emplace_back(track->first);
			else //predict forward
			{
				track->second.statePrediction(fusionTime, parameters.largeProcessNoiseStd());
				track->second.writeTimeStamp(fusionTime);
			}
		}
		else //exist associated tracks according to association table
		{
			pair<VectorXd, MatrixXd> fusedState(states.begin()->second.first, states.begin()->second.second);
			if (states.size() > 1)
				for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator state = ++states.begin(); state != states.end(); state++)
					TrackAssociation::covarianceIntersection(fusedState.first, fusedState.second, state->second.first, state->second.second, fusedState);
			track->second.statePrediction(fusionTime, parameters.largeProcessNoiseStd());
			//update fused track with fused position 
			VectorXd meas;
			if (parameters.getIndices().size() == 5)
			{
				meas = VectorXd::Zero(5);
				meas.head(3) = fusedState.first({ 0,3,6 });
				meas.tail(2) = fusedState.first({ 1,4 });
			}
			else if (parameters.getIndices().size() == 3)
				meas = fusedState.first({ 0,3,6 });
			track->second.stateUpdate(parameters.H(), parameters.R(), meas, fusionTime);
			//or directly replace preidicted state with fused state
			//track->second.replaceLastState(fusedState.first, fusedState.second);
		}

	}
	for (uint32_t& id : terminatedTrackIDs) // delete termianted tracks from survived tracks
	{
		this->m_terminatedTrack.emplace(pair<uint32_t, Track>(id, this->m_track[id]));
		this->m_track.erase(id);
	}
}

void FusedTrack::statePrediction(double fusionTime, double sigma, map<uint32_t, pair<VectorXd, MatrixXd>>& states)
{
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
	{
		pair<VectorXd, MatrixXd> priorState = track->second.getCurrentState();
		track->second.statePrediction(fusionTime, sigma);
		pair<VectorXd, MatrixXd> state = track->second.getCurrentState();
		bool nan = isnan(state.first(0));
		if (nan)
		{
			cout << "prior state = " << priorState.first.transpose() << endl;
			cout << "predicted state = " << state.first.transpose() << endl;
		}
		states.emplace(pair<uint32_t, pair<VectorXd, MatrixXd>>(track->first, state));
	}
}

void FusedTrack::stateUpdate(Parameters& parameters, double fusionTime, map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& states)
{
	vector<uint32_t> terminatedTrackIDs;
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
	{
		map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator found = states.find(track->first);
		if (found != states.end())
			if (!states[track->first].empty())
			{
				pair<VectorXd, MatrixXd> priorState = track->second.getCurrentState();
				//cout << "****** fused track prior state = " << priorState.first(parameters.getIndices()).transpose() << endl;
				pair<VectorXd, MatrixXd> fusedState(states[track->first].begin()->second.first, states[track->first].begin()->second.second);
				if (states[track->first].size() > 1)
				{
					map<uint32_t, pair<VectorXd, MatrixXd>>::iterator iter = states[track->first].begin();
					for (iter++; iter != states[track->first].end(); iter++)
					{
						//cout << "node track state = " << iter->second.first(parameters.getIndices()).transpose() << endl;
						TrackAssociation::covarianceIntersection(fusedState.first, fusedState.second,
							iter->second.first, iter->second.second, fusedState);
					}
				}
				//cout << "******covariance intersection state = " << fusedState.first(parameters.getIndices()).transpose() << endl;
				VectorXd meas;
				if (parameters.getIndices().size() == 5)
				{
					meas = VectorXd::Zero(5);
					meas.head(3) = fusedState.first({ 0,3,6 });
					meas.tail(2) = fusedState.first({ 1,4 });
				}
				else if(parameters.getIndices().size() == 3)
					meas = fusedState.first({ 0,3,6 });
				track->second.stateUpdate(parameters.H(), parameters.R(), meas, fusionTime);
				pair<VectorXd, MatrixXd> posteriorState = track->second.getCurrentState();
				//cout << "******posterior state = " << posteriorState.first(parameters.getIndices()).transpose() << endl;
			}
			else
				if (fusionTime - track->second.getLastUpdateTime() > parameters.fusedTrackTerminateTime())
				{
					track->second.writeSrurviveStatus(false);
					terminatedTrackIDs.emplace_back(track->first);
				}
				else
					track->second.writeTimeStamp(fusionTime);
		else
		{
			if (fusionTime - track->second.getLastUpdateTime() > parameters.fusedTrackTerminateTime())
			{
				track->second.writeSrurviveStatus(false);
				terminatedTrackIDs.emplace_back(track->first);
			}
			else 
				track->second.writeTimeStamp(fusionTime);
		}
	}
	for (uint32_t& id : terminatedTrackIDs)
	{
		this->m_terminatedTrack.emplace(pair<uint32_t, Track>(id, this->m_track[id]));
		this->m_track.erase(id);
	}
}

void FusedTrack::initializeAssociatedState(map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
		associatedStates.emplace(pair<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>(track->first, map<uint32_t, pair<VectorXd, MatrixXd>>()));
}

void FusedTrack::writeAssociationInfo(uint32_t groupID, uint32_t nodeTrackID, uint32_t fusedTrackID)
{
	map<uint32_t, map<uint32_t, uint32_t>>::iterator foundNodeTrackID = this->m_numAssociations[groupID].find(nodeTrackID);
	if (foundNodeTrackID != this->m_numAssociations[groupID].end())
	{
		map<uint32_t, uint32_t>::iterator foundFusedTrackID = foundNodeTrackID->second.find(fusedTrackID);
		if (foundFusedTrackID != foundNodeTrackID->second.end())
		{
			this->m_numAssociations[groupID][nodeTrackID][fusedTrackID]++;
			//if (this->m_numAssociations[groupID][nodeTrackID][fusedTrackID] >= 10)
			//{
			//	//cout << "group id = " << groupID << ", node track id = " << nodeTrackID << ", fused tarck id = " << fusedTrackID << endl;
			//	//cout << "***********************************association to be confirmed" << endl;
			//}
		}
		else
		{
			pair<uint32_t, uint32_t> association(fusedTrackID, 1);
			pair<uint32_t, map<uint32_t, uint32_t>> nodeAssociation;
			nodeAssociation.first = nodeTrackID;
			nodeAssociation.second.emplace(association);
			this->m_numAssociations[groupID].emplace(nodeAssociation);
		}
	}
	else
	{
		map<uint32_t, uint32_t> associations;
		associations.emplace(pair<uint32_t, uint32_t>(fusedTrackID, 1));
		this->m_numAssociations[groupID].emplace(pair<uint32_t, map<uint32_t, uint32_t>>(nodeTrackID, associations));
	}
}

void FusedTrack::writeAssociation(uint32_t groupID, uint32_t nodeTrackID, uint32_t fusedTrackID, map<uint32_t, map<uint32_t, uint32_t>>& associations)
{
	map<uint32_t, map<uint32_t, uint32_t>>::iterator found = associations.find(fusedTrackID);
	if (found != associations.end())
		associations[fusedTrackID].emplace(pair<uint32_t, uint32_t>(nodeTrackID, groupID));
	else
	{
		map<uint32_t, uint32_t> association;
		association.emplace(pair<uint32_t, uint32_t>(nodeTrackID, groupID));
		associations.emplace(pair<uint32_t, map<uint32_t, uint32_t>>(fusedTrackID, association));
	}
}

void FusedTrack::writeAssociation(pair<double, map<uint32_t, map<uint32_t, uint32_t>>> associations)
{
	this->m_associations.emplace(associations);
	for (map<uint32_t, map<uint32_t, uint32_t>>::iterator fusedTrack = associations.second.begin(); fusedTrack != associations.second.end(); fusedTrack++)
	{
		map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>::iterator foundFusedTrack = this->m_associationHistory.find(fusedTrack->first);
		if (foundFusedTrack != this->m_associationHistory.end())
		{
			for (map<uint32_t, uint32_t>::iterator nodeTrack = fusedTrack->second.begin(); nodeTrack != fusedTrack->second.end(); nodeTrack++)
			{
				map<uint32_t, map<uint32_t, uint32_t>>::iterator foundGroup = foundFusedTrack->second.find(nodeTrack->second);
				if (foundGroup != foundFusedTrack->second.end())
				{
					map<uint32_t, uint32_t>::iterator foundNodeTrack = foundGroup->second.find(nodeTrack->first);
					if (foundNodeTrack != foundGroup->second.end())
						this->m_associationHistory[foundFusedTrack->first][foundGroup->first][foundNodeTrack->first]++;
					else
						this->m_associationHistory[foundFusedTrack->first][foundGroup->first].emplace(pair<uint32_t, uint32_t>(nodeTrack->first, 1));
				}
				else
				{
					map<uint32_t, uint32_t> newAssociation;
					newAssociation.emplace(pair<uint32_t, uint32_t>(nodeTrack->first, 1));
					pair<uint32_t, map<uint32_t, uint32_t>> newGroup(nodeTrack->second, newAssociation);
					this->m_associationHistory[foundFusedTrack->first].emplace(newGroup);
				}
			}
		}
		else
		{
			map<uint32_t, map<uint32_t, uint32_t>> numAssociation;
			for (map<uint32_t, uint32_t>::iterator nodeTrack = fusedTrack->second.begin(); nodeTrack != fusedTrack->second.end(); nodeTrack++)
				numAssociation[nodeTrack->second].emplace(pair<uint32_t, uint32_t>(nodeTrack->first, 1));
			for(map<uint32_t, map<uint32_t, uint32_t>>::iterator iter = numAssociation.begin(); iter!= numAssociation.end(); iter++)
				this->m_associationHistory[fusedTrack->first].emplace(*iter);
		}
	}
}

void FusedTrack::updateAssociationTable(uint32_t threshold)
{
	vector<pair<uint32_t, pair<uint32_t, uint32_t>>> confirmedPairs; //group->node track id->fused track id
	//write confirmed association pair into associationTable
	for (map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>::iterator group = this->m_numAssociations.begin(); group != this->m_numAssociations.end(); group++)
	{
		//cout << "group id = " << group->first << endl;
		for (map<uint32_t, map<uint32_t, uint32_t>>::iterator nodeTrack = group->second.begin(); nodeTrack != group->second.end(); nodeTrack++)
		{
			if (!nodeTrack->second.empty())
			{
				//cout << "node track id = " << nodeTrack->first << endl;
				vector<uint32_t> numAssociations;
				map<uint32_t, uint32_t>::iterator association = nodeTrack->second.begin();
				for (; association != nodeTrack->second.end(); association++)
					numAssociations.emplace_back(association->second);
				vector<uint32_t>::iterator maxPosition = max_element(numAssociations.begin(), numAssociations.end());
				int index = maxPosition - numAssociations.begin();
				association = nodeTrack->second.begin();
				advance(association, index);
				if (association->second >= threshold)
				{
					uint32_t nodeTrackID = nodeTrack->first;
					set<uint32_t>::iterator foundAssociatedTrack = this->m_associationConfirmedTrackID[group->first].find(nodeTrackID);
					if (foundAssociatedTrack == this->m_associationConfirmedTrackID[group->first].end())
					{
						uint32_t fusedTrackID = association->first;
						//cout << "############# fused track " << fusedTrackID << " associates with node track " << nodeTrackID << endl;
						pair<uint32_t, uint32_t> id(nodeTrackID, fusedTrackID);
						confirmedPairs.emplace_back(pair<uint32_t, pair<uint32_t, uint32_t>>(group->first, id));
						map<uint32_t, map<uint32_t, uint32_t>>::iterator found = this->m_associationTable.find(fusedTrackID);
						if (found != this->m_associationTable.end())
							this->m_associationTable[fusedTrackID].emplace(pair<uint32_t, uint32_t>(nodeTrackID, group->first));
						else
							this->m_associationTable[fusedTrackID].emplace(pair<uint32_t, uint32_t>(nodeTrackID, group->first));
						this->m_associationConfirmedTrackID[group->first].emplace(nodeTrackID);
					}
				}
			}
		}
	}
	//delete confirmed association pair from numAssociation
	for (pair<uint32_t, pair<uint32_t, uint32_t>>& id : confirmedPairs)
		this->m_numAssociations[id.first][id.second.first].erase(id.second.second);
}

void FusedTrack::getAssociatedTrackID(uint32_t id, pair<bool, uint32_t>& association)
{
	map<uint32_t, uint32_t> numAssociations;
	for (map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>::iterator group = this->m_numAssociations.begin(); group != this->m_numAssociations.end(); group++)
	{
		map<uint32_t, map<uint32_t, uint32_t>>::iterator found = group->second.find(id);
		if (found != group->second.end())
		{
			for (map<uint32_t, uint32_t>::iterator nodeTrackID = found->second.begin(); nodeTrackID != found->second.end(); nodeTrackID++)
				numAssociations.emplace(pair<uint32_t, uint32_t>(nodeTrackID->second, nodeTrackID->first));
			if (!numAssociations.empty())
			{
				association.first = true;
				association.second = numAssociations.rbegin()->second;
			}
		}
	}
}

void FusedTrack::getAssociationHistory(map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>& history)
{
	map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>::iterator fusedTrack = this->m_associationHistory.begin();
	for (; fusedTrack != this->m_associationHistory.end(); fusedTrack++)
		history.emplace(*fusedTrack);
}

void FusedTrack::writeSources(pair<uint32_t, map<uint32_t, uint32_t>>& sources)
{
	this->m_sources.emplace(sources);
}

void FusedTrack::getSurvivedTrack(map<uint32_t, Track>& tracks)
{
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
		tracks.emplace(*track);
}

void FusedTrack::getTerminatedTrack(map<uint32_t, Track>& tracks)
{
	for (map<uint32_t, Track>::iterator track = this->m_terminatedTrack.begin(); track != this->m_terminatedTrack.end(); track++)
		tracks.emplace(*track);
}

vector<uint32_t> FusedTrack::getUnassociatedTrackID(vector<uint32_t> associatedTrackIDs)
{
	vector<uint32_t> unassociatedTrackIDs;
	for (map<uint32_t, Track>::iterator iter = this->m_track.begin(); iter != this->m_track.end(); iter++)
	{
		pair<bool, size_t> found = Functions::findIndex(iter->first, associatedTrackIDs);
		if (!found.first)
			unassociatedTrackIDs.emplace_back(iter->first);
	}
	return unassociatedTrackIDs;
}

map<uint32_t, vector<VectorXd>> FusedTrack::getTracks()
{
	map<uint32_t, vector<VectorXd>> tracks;
	for (map<uint32_t, Track>::iterator track = this->m_track.begin(); track != this->m_track.end(); track++)
	{
		vector<VectorXd> states;
		track->second.getState(states);
		tracks.emplace(pair<uint32_t, vector<VectorXd>>(track->first, states));
	}
	for (map<uint32_t, Track>::iterator track = this->m_terminatedTrack.begin(); track != this->m_terminatedTrack.end(); track++)
	{
		vector<VectorXd> states;
		track->second.getState(states);
		tracks.emplace(pair<uint32_t, vector<VectorXd>>(track->first, states));
	}
	return tracks;
}

pair<VectorXd, MatrixXd> FusedTrack::getLatestTrackState(uint32_t trackID)
{
	return this->m_track[trackID].getCurrentState();
}

HierarchicalRadarFusion::HierarchicalRadarFusion() 
{
	this->m_numGroups = 0;
	this->m_numHierarchies = 0;
};

HierarchicalRadarFusion::~HierarchicalRadarFusion() {};

void HierarchicalRadarFusion::getRadarID(string fileName)
{
	ifstream ifs;
	ifs.open(fileName, ios::in);
	string line;
	while (getline(ifs, line)) this->m_radarID.emplace_back(atoi(line.c_str()));
	//for (int id : this->m_radarID) cout << id << endl;
	ifs.close();
}

void HierarchicalRadarFusion::assignRadarGroup(const uint32_t numRadarsOfaGroup, uint32_t groupIDFactor)
{
	size_t numRadars = this->m_radarID.size();
	int numGroups = ceil((double)numRadars / numRadarsOfaGroup);
	if (numRadars % numRadarsOfaGroup == 1) numGroups--;
	for (size_t i = 0; i < numRadars; i++)
	{
		int group = i / numRadarsOfaGroup;
		if (numRadars % numRadarsOfaGroup == 1 && i == numRadars - 1) group--;
		this->m_radarGroup.emplace(pair<int, int>(this->m_radarID[i], (group + 1) * groupIDFactor));
	}
	this->m_numGroups = numGroups;
	/*for (map<uint32_t, uint32_t>::iterator iter = this->m_radarGroup.begin(); iter != this->m_radarGroup.end(); iter++)
	{
		cout << "id = " << iter->first << '\t' << "group = " << iter->second << endl;
	}*/
}

uint32_t HierarchicalRadarFusion::numGroups() const
{
	return this->m_numGroups;
}

void HierarchicalRadarFusion::initializeRadarTrackInfo()
{
	for (map<uint32_t, uint32_t>::iterator iter = this->m_radarGroup.begin(); iter != this->m_radarGroup.end(); iter++)
		this->m_radarTrack[iter->second].emplace(pair<int, RadarTrack>(iter->first, RadarTrack()));
}

void HierarchicalRadarFusion::wrapReceivedData(queue<map<uint32_t, ReceivedRadarTrack>>& data)
{
	while (!data.empty())
	{
		map<uint32_t, ReceivedRadarTrack> receivedSingleStepData = data.front();
		map<uint32_t, map<uint32_t, ReceivedRadarTrack>> wrappedDingleStepData;
		for (map<uint32_t, ReceivedRadarTrack>::iterator iter = receivedSingleStepData.begin(); iter != receivedSingleStepData.end(); iter++)
		{
			map<uint32_t, uint32_t>::iterator iterFind = this->m_radarGroup.find(iter->first);
			if (iterFind != this->m_radarGroup.end())
				wrappedDingleStepData[iterFind->second].emplace(*iter);
		}
		this->m_receivedData.push(wrappedDingleStepData);
		data.pop();
	}
}

//for multiple threads
void HierarchicalRadarFusion::parallelRadarTrackFiltering(Parameters parameters, pair<uint32_t, map<uint32_t, ReceivedRadarTrack>> wrappedData)
{
	uint32_t group = wrappedData.first;
	map<uint32_t, ReceivedRadarTrack> data = wrappedData.second;
	for (map<uint32_t, ReceivedRadarTrack>::iterator iter = data.begin(); iter != data.end(); iter++)
		this->m_radarTrack[group][iter->first].radarTrackFiltering(parameters, iter->second);
}

void HierarchicalRadarFusion::RadarTrackFiltering(Parameters& parameters, ReceivedData& receivedData)
{
	queue<map<uint32_t, ReceivedRadarTrack>> data;
	receivedData.readData(data);
	this->wrapReceivedData(data);
	while (!this->m_receivedData.empty())
	{
		map<uint32_t, map<uint32_t, ReceivedRadarTrack>> singleStepData = this->m_receivedData.front();
		if (singleStepData.size() == 1) 
		{
			this->parallelRadarTrackFiltering(parameters, *singleStepData.begin());
		}
		else if (singleStepData.size() > 1) //multiple groups -> call multiple threads
		{
			vector<thread> threads;
			for (map<uint32_t, map<uint32_t, ReceivedRadarTrack>>::iterator iter = singleStepData.begin(); iter != singleStepData.end(); iter++)
			{
				thread threadParallelFiltering(&HierarchicalRadarFusion::parallelRadarTrackFiltering, this, parameters, *iter);
				threads.emplace_back(move(threadParallelFiltering));
			}
			for (thread& t : threads) t.join();
		}
		this->m_receivedData.pop();
	}
}

void HierarchicalRadarFusion::initializeHierarchicalFusedTrack(const uint32_t numMembers, uint32_t hierarchyIDFactor, uint32_t groupIDFactor)
{
	int numHierarchies = ceil((double)this->m_numGroups / numMembers);
	if (this->m_numGroups > numMembers && this->m_numGroups % numMembers == 1) numHierarchies--;
	this->m_numHierarchies = numHierarchies;
	for (int h = 0; h < numHierarchies; h++)
		this->m_hierarchicalFusedTrack.emplace(pair<uint32_t, map<uint32_t, FusedTrack>>((h + 1) * hierarchyIDFactor, map<uint32_t, FusedTrack>()));
	map<uint32_t, map<uint32_t, FusedTrack>>::iterator iter = this->m_hierarchicalFusedTrack.begin();
	uint32_t baseHierarchy = iter->first;
	for (int g = 0; g < this->m_numGroups; g++)
	{
		vector<uint32_t> ids;
		for (map<uint32_t, uint32_t>::iterator iter = this->m_radarGroup.begin(); iter != this->m_radarGroup.end(); iter++)
			if (iter->second == (g + 1) * groupIDFactor)
				ids.emplace_back(iter->first);
		//FusedTrack fusedTrack(ids);
		this->m_hierarchicalFusedTrack[baseHierarchy].emplace(pair<int, FusedTrack>((g + 1) * groupIDFactor, FusedTrack(ids)));
	}
	if (numHierarchies > 1)
		for (int h = 1; h < numHierarchies; h++)
		{
			iter = this->m_hierarchicalFusedTrack.begin();
			advance(iter, h);
			uint32_t currentHierarchy = iter->first;
			advance(iter, -1);
			uint32_t previousHierarchy = iter->first;
			int numGroups = ceil((double)this->m_hierarchicalFusedTrack[previousHierarchy].size() / numMembers);
			if (this->m_hierarchicalFusedTrack[previousHierarchy].size() % numMembers == 1) numGroups--;
			for (int g = 0; g < numGroups; g++)
			{
				vector<uint32_t> ids;
				for (uint32_t m = 0; m < this->m_hierarchicalFusedTrack[previousHierarchy].size(); m++)
				{
					if (m / numMembers == g)
						ids.emplace_back((m + 1) * groupIDFactor);
					if(numGroups == 1 && m % numMembers == 1 && m == this->m_hierarchicalFusedTrack[previousHierarchy].size() - 1)
						ids.emplace_back((m + 1) * groupIDFactor);
				}
				//FusedTrack fusedTrack(ids);
				this->m_hierarchicalFusedTrack[currentHierarchy].emplace(pair<int, FusedTrack>((g + 1) * groupIDFactor, FusedTrack(ids)));
				this->m_subheirarchyIDs[currentHierarchy].emplace(pair<uint32_t, vector<uint32_t>>((g + 1) * groupIDFactor, ids));
			}
		}
}

void HierarchicalRadarFusion::getRadarTrackInfo(Parameters& parameters, double fusionTime, bool useSystemTrack,
	map<uint32_t, FusedTrack>::iterator hierarchicalFusedTrack,
	map<uint32_t, map<uint32_t, uint32_t>>& associations,
	map<uint32_t, pair<VectorXd, MatrixXd>>& fusedTrackStates,
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& nodeTrackStates,
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	uint32_t group = hierarchicalFusedTrack->first;
	FusedTrack* fusedTracks;
	if (useSystemTrack)
		fusedTracks = &(this->m_systemTrack);
	else
		fusedTracks = &(hierarchicalFusedTrack->second);
	//FusedTrack& fusedTracks = hierarchicalFusedTrack->second;
	map<uint32_t, RadarTrack>& radarTracks = this->m_radarTrack[group];
	fusedTracks->statePrediction(fusionTime, parameters.largeProcessNoiseStd(), fusedTrackStates);
	map<uint32_t, vector<size_t>> unassociatedNodeTrackIndices, associatedNodeTrackIndices; //radar id ->(un)associated track indices
	fusedTracks->initializeAssociatedState(associatedStates);
	//get tracks in the association table
	fusedTracks->getInfoFromAssociationTable(parameters, fusionTime, radarTracks, associations, associatedNodeTrackIndices, associatedStates);
	//get the indices of unassociated tracks
	for (map<uint32_t, RadarTrack>::iterator radarTrack = radarTracks.begin(); radarTrack != radarTracks.end(); radarTrack++)
	{
		vector<size_t> survivedTrackIndex;
		radarTrack->second.getSurvivedTrackIndex(survivedTrackIndex);
		for (size_t i = 0; i < survivedTrackIndex.size(); i++)
			if (!Functions::isMember(survivedTrackIndex[i], associatedNodeTrackIndices[radarTrack->first]))
				unassociatedNodeTrackIndices[radarTrack->first].emplace_back(survivedTrackIndex[i]);
	}
	//align states to fusion time
	for (map<uint32_t, RadarTrack>::iterator radarTrack = radarTracks.begin(); radarTrack != radarTracks.end(); radarTrack++)
	{
		uint32_t radarID = radarTrack->first;
		if (!unassociatedNodeTrackIndices[radarID].empty())
			for (size_t index : unassociatedNodeTrackIndices[radarID])
			{
				pair<VectorXd, MatrixXd> extrapolate = radarTrack->second.extrapolateTrack(parameters, fusionTime, index);
				nodeTrackStates[radarID].emplace(pair<uint32_t, pair<VectorXd, MatrixXd>>(radarTrack->second.getTrackID(index), extrapolate));
			}
	}
}

void HierarchicalRadarFusion::groupTrackFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy, uint32_t group,
	FusedTrack* fusedTracks, 
	map<uint32_t, map<uint32_t, uint32_t>>& associations,
	map<uint32_t, pair<VectorXd, MatrixXd>>& fusedTrackStates, 
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& nodeTrackStates,
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>& associatedStates)
{
	map<uint32_t, vector<uint32_t>> associatedTrackIDs; //radar id and associated tarck ids
	for (map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator iter = nodeTrackStates.begin(); iter != nodeTrackStates.end(); iter++)
		associatedTrackIDs.emplace(pair<uint32_t, vector<uint32_t>>(iter->first, vector<uint32_t>()));
	//sequential association of fused tracks and node tracks 
	if (!fusedTrackStates.empty()) //(state maybe nan)应该加上不和同源关联的逻辑
	{
		map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator state;
		for (state = nodeTrackStates.begin(); state != nodeTrackStates.end(); state++)
			if (!state->second.empty())
			{
				vector<uint32_t> ids;
				vector<int> assignment(fusedTrackStates.size(), parameters.invalidAssignment());
				TrackAssociation::radarTrackAssociation(parameters, fusedTrackStates, state->second, assignment);
				for (size_t i = 0; i < assignment.size(); i++)
					if (assignment[i] != parameters.invalidAssignment())
					{
						map<uint32_t, pair<VectorXd, MatrixXd>>::iterator iter = fusedTrackStates.begin();
						advance(iter, i);
						uint32_t fusedTrackID = iter->first;
						iter = state->second.begin();
						advance(iter, assignment[i]);
						uint32_t nodeTrackID = iter->first;
						associatedStates[fusedTrackID].emplace(pair<uint32_t, pair<VectorXd, MatrixXd>>(nodeTrackID, state->second[nodeTrackID]));
						ids.emplace_back(nodeTrackID);
						uint32_t radarID = state->first;
						fusedTracks->writeAssociationInfo(radarID, nodeTrackID, fusedTrackID);
						//if(group == 1) cout << "fused track " << fusedTrackID << " associates with node track " << nodeTrackID << endl;
						fusedTracks->writeAssociation(radarID, nodeTrackID, fusedTrackID, associations);
					}
				associatedTrackIDs[state->first].assign(ids.begin(), ids.end());
			}
	}
	//get states of unassociated tracks 
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> unassociatedStates; //
	for (map<uint32_t, vector<uint32_t>>::iterator index = associatedTrackIDs.begin(); index != associatedTrackIDs.end(); index++)
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator state = nodeTrackStates[index->first].begin(); state != nodeTrackStates[index->first].end(); state++)
			if (!Functions::isMember(state->first, index->second))
				unassociatedStates[index->first].emplace(*state);
	//未与融合航迹关联上的节点航迹遍历找到关联次数最多的融合航迹关联，没有关联过再起始新的融合航迹
	for (map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator radar = unassociatedStates.begin();
		radar != unassociatedStates.end(); radar++)
	{
		vector<uint32_t> prunedIDs;
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator track = radar->second.begin(); track != radar->second.end(); track++)
		{
			pair<bool, uint32_t> association(false, 0);
			fusedTracks->getAssociatedTrackID(track->first, association);
			if (association.first)
			{
				uint32_t fusedTrackID = association.second;
				map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator found = associatedStates.find(fusedTrackID);
				associatedStates[fusedTrackID].emplace(*track);
				prunedIDs.emplace_back(track->first);
			}
		}
		for (uint32_t& id : prunedIDs)
			radar->second.erase(id);
	}
	//fused track update and termiantion
	fusedTracks->stateUpdate(parameters, fusionTime, associatedStates);
	//sequential association of unassociated tracks for initialization of fused tarcks (for unassociatedStates.size() > 1)
	if (unassociatedStates.size() == 1) //direct initialization
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator iter = unassociatedStates.begin()->second.begin(); iter != unassociatedStates.begin()->second.end(); iter++)
		{
			size_t numTracks = fusedTracks->getNumTracks() + fusedTracks->getNumTerminatedTracks();
			uint32_t newTrackID = hierarchy + group + numTracks + 1;
			fusedTracks->initializeNewFusedTrack(newTrackID, fusionTime, iter->second.first, iter->second.second);
			fusedTracks->writeAssociationInfo(unassociatedStates.begin()->first, iter->first, newTrackID);
			//cout << "2 writeAssociationInfo radarID = " << unassociatedStates.begin()->first << endl;
			fusedTracks->writeAssociation(unassociatedStates.begin()->first, iter->first, newTrackID, associations);
			map<uint32_t, uint32_t> idPairs;
			idPairs.emplace(pair<uint32_t, uint32_t>(iter->first, unassociatedStates.begin()->first));
			pair<uint32_t, map<uint32_t, uint32_t>> origin(newTrackID, idPairs);
			fusedTracks->writeSources(origin);
		}
	else if (unassociatedStates.size() > 1) //association then initialization
	{
		map<uint32_t, pair<VectorXd, MatrixXd>> referedState;
		map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>>::iterator states = unassociatedStates.begin();
		for (; states != unassociatedStates.end(); states++)
			if (!states->second.empty())
			{
				referedState = states->second;
				break;
			}
		map<uint32_t, uint32_t> referedID; // track id (unique keys) and group id
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator iter = referedState.begin(); iter != referedState.end(); iter++)
		{
			referedID.emplace(pair<uint32_t, uint32_t>(iter->first, states->first));
			//cout << "1 refered id -> radar id = " << states->first << endl;
		}
		map<uint32_t, vector<int>> assignments; //associated radar id, <radar id, assignment>>
		map<uint32_t, map<uint32_t, uint32_t>> associationInfo; //track id-> associated track id ->group id
		//track association
		if (!referedState.empty())
			for (states++; states != unassociatedStates.end(); states++)
				if (!states->second.empty())
				{
					vector<int> assignment(referedState.size(), parameters.invalidAssignment());
					TrackAssociation::radarTrackAssociation(parameters, referedState, states->second, assignment);
					assignments.emplace(pair<uint32_t, vector<int>>(states->first, assignment));
					vector<uint32_t> associatedIndices;
					for (size_t i = 0; i < assignment.size(); i++)
						if (assignment[i] != parameters.invalidAssignment())
						{
							associatedIndices.emplace_back((uint32_t)assignment[i]);
							map<uint32_t, pair<VectorXd, MatrixXd>>::iterator state = referedState.begin();
							advance(state, i);
							uint32_t referedTrackID = state->first;
							map<uint32_t, map<uint32_t, uint32_t>>::iterator found = associationInfo.find(referedTrackID);
							state = states->second.begin();
							advance(state, assignment[i]);
							uint32_t associatedTrackID = state->first;
							if (found != associationInfo.end())
								associationInfo[referedTrackID].emplace(pair<uint32_t, uint32_t>(associatedTrackID, states->first));
							else
							{
								map<uint32_t, uint32_t> id;
								id.emplace(pair<uint32_t, uint32_t>(associatedTrackID, states->first));
								associationInfo.emplace(pair<uint32_t, map<uint32_t, uint32_t>>(referedTrackID, id));
							}
						}
					for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator state = states->second.begin(); state != states->second.end(); state++)
					{
						uint32_t index = distance(states->second.begin(), state);
						if (!Functions::isMember(index, associatedIndices))
						{
							referedState.emplace(*state);
							referedID.emplace(pair<uint32_t, uint32_t>(state->first, states->first));
							//cout << "2 refered id -> radar id = " << states->first << endl;
						}
					}
				}
		//extract association information from associationInfo
		vector<vector<pair<uint32_t, uint32_t>>> wrappedIDs; //track id and group id
		vector<vector<pair<VectorXd, MatrixXd>>> wrappedStates;
		for (map<uint32_t, pair<VectorXd, MatrixXd>>::iterator state = referedState.begin(); state != referedState.end(); state++)
		{
			int index = distance(referedState.begin(), state);
			map<uint32_t, uint32_t>::iterator id = referedID.begin();
			advance(id, index);
			vector<pair<uint32_t, uint32_t>> associatedTrackIDs;
			associatedTrackIDs.emplace_back(*id);
			vector<pair<VectorXd, MatrixXd>> associatedTrackStates;
			associatedTrackStates.emplace_back(state->second);
			map<uint32_t, map<uint32_t, uint32_t>>::iterator found = associationInfo.find(state->first);
			if (found != associationInfo.end())
				for (id = found->second.begin(); id != found->second.end(); id++)
				{
					associatedTrackIDs.emplace_back(*id);
					associatedTrackStates.emplace_back(unassociatedStates[id->second][id->first]);
				}
			wrappedIDs.emplace_back(associatedTrackIDs);
			wrappedStates.emplace_back(associatedTrackStates);
		}
		//fused track initialization
		for (size_t t = 0; t < wrappedIDs.size(); t++)
		{
			size_t numTracks = fusedTracks->getNumTracks() + fusedTracks->getNumTerminatedTracks();
			uint32_t newTrackID = hierarchy + group + numTracks + 1;
			if (wrappedIDs[t].size() == 1)
			{
				fusedTracks->initializeNewFusedTrack(newTrackID, fusionTime, wrappedStates[t].front().first, wrappedStates[t].front().second);
				fusedTracks->writeAssociationInfo(wrappedIDs[t].front().second, wrappedIDs[t].front().first, newTrackID);
				//cout << "3 writeAssociationInfo radarID = " << wrappedIDs[t].front().second << endl;
				fusedTracks->writeAssociation(wrappedIDs[t].front().second, wrappedIDs[t].front().first, newTrackID, associations);
				map<uint32_t, uint32_t> idPairs;
				idPairs.emplace(pair<uint32_t, uint32_t>(wrappedIDs[t].front().first, wrappedIDs[t].front().second));
				pair<uint32_t, map<uint32_t, uint32_t>> origin(newTrackID, idPairs);
				fusedTracks->writeSources(origin);
			}
			else if (wrappedIDs[t].size() > 1)
			{
				pair<VectorXd, MatrixXd> fusedState(wrappedStates[t].front().first, wrappedStates[t].front().second);
				for (size_t i = 1; i < wrappedStates[t].size(); i++)
					TrackAssociation::covarianceIntersection(fusedState.first, fusedState.second,
						wrappedStates[t][i].first, wrappedStates[t][i].second, fusedState);
				fusedTracks->initializeNewFusedTrack(newTrackID, fusionTime, fusedState.first, fusedState.second);
				map<uint32_t, uint32_t> idPairs;
				for (pair<uint32_t, uint32_t>& id : wrappedIDs[t])
				{
					fusedTracks->writeAssociationInfo(id.second, id.first, newTrackID);
					//cout << "4 writeAssociationInfo radarID = " << id.second << endl;
					fusedTracks->writeAssociation(id.second, id.first, newTrackID, associations);
					idPairs.emplace(pair<uint32_t, uint32_t>(id.first, id.second));
				}
				pair<uint32_t, map<uint32_t, uint32_t>> origin(newTrackID, idPairs);
				fusedTracks->writeSources(origin);
			}
		}
	}
	//update association table and write association info into m_associations (需要保存关联信息)
	fusedTracks->updateAssociationTable(parameters.thresholdConfirmedAssociation());
	fusedTracks->writeAssociation(pair<double, map<uint32_t, map<uint32_t, uint32_t>>>(fusionTime, associations));
	/*if (group == 1)
	{
		cout << "$$$$$$$$$$$$$$$$$$$$$ association information 2 $$$$$$$$$$$$$$$$$$$$$$$$$$$$" << endl;
		for (map<uint32_t, map<uint32_t, uint32_t>>::iterator iter = associations.begin(); iter != associations.end(); iter++)
		{
			cout << "fused track id = " << iter->first << endl;
			for (map<uint32_t, uint32_t>::iterator id = iter->second.begin(); id != iter->second.end(); id++)
			{
				cout << "node track id = " << id->first << endl;
			}
		}
	}*/
}

// pair<uint32_t, FusedTrack> group index and fused tracks of this group (for multiple threads)
void HierarchicalRadarFusion::parallelRadarTrackFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy, 
	map<uint32_t, FusedTrack>::iterator hierarchicalFusedTrack, bool useSystemTrack)
{
	map<uint32_t, map<uint32_t, uint32_t>> associations;
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> nodeTrackStates; //radar id->track id->state (tracks not in the association table) 
	map<uint32_t, pair<VectorXd, MatrixXd>> fusedTrackStates; //fused track state prediction
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> associatedStates; //fused track id->associated track id->Gaussian (to update fused tracks)
	this->getRadarTrackInfo(parameters, fusionTime, useSystemTrack, hierarchicalFusedTrack, associations, fusedTrackStates, nodeTrackStates, associatedStates);
	uint32_t group = hierarchicalFusedTrack->first;
	FusedTrack* fusedTracks;
	if (useSystemTrack)
	{
		fusedTracks = &(this->m_systemTrack);
		group = 0;
		hierarchy = 0;
	}
	else
		fusedTracks = &(hierarchicalFusedTrack->second);
	this->groupTrackFusion(parameters, fusionTime, hierarchy, group, fusedTracks, associations, fusedTrackStates, nodeTrackStates, associatedStates);
	//cout << "end of parallelRadarTrackFusion" << endl;
}

void HierarchicalRadarFusion::rootHierarchyFusion(Parameters& parameters, double fusionTime, bool useSystemTrack)
{
	uint32_t hierarchy = this->m_hierarchicalFusedTrack.begin()->first;
	bool multipleThreads = true;
	if (multipleThreads)
	{
		vector<thread> threads;
		for (map<uint32_t, FusedTrack>::iterator tracks = this->m_hierarchicalFusedTrack.begin()->second.begin();
			tracks != this->m_hierarchicalFusedTrack.begin()->second.end(); tracks++)
		{
			thread threadParallelRadarFusion(&HierarchicalRadarFusion::parallelRadarTrackFusion, this, ref(parameters), 
				fusionTime, hierarchy, tracks, useSystemTrack);
			threads.emplace_back(move(threadParallelRadarFusion));
		}
		for (thread& t : threads) t.join();
	}
	else
	{
		for (map<uint32_t, FusedTrack>::iterator tracks = this->m_hierarchicalFusedTrack.begin()->second.begin();
			tracks != this->m_hierarchicalFusedTrack.begin()->second.end(); tracks++)
		{
			//if(tracks->first == 1 || tracks->first == 2) cout << "*************group " << tracks->first << endl;
			this->parallelRadarTrackFusion(parameters, fusionTime, hierarchy, tracks, useSystemTrack);
		}
	}
}

void HierarchicalRadarFusion::middleHierarchyFusion(Parameters& parameters, double fusionTime, uint32_t hierarchy, 
	map<uint32_t, FusedTrack>::iterator currentHierarchyTrack, map<uint32_t, FusedTrack>& lastHierarchyTrack, vector<uint32_t> subgroupIDs)
{
	uint32_t group = currentHierarchyTrack->first;
	FusedTrack* fusedTracks = &(currentHierarchyTrack->second);
	map<uint32_t, map<uint32_t, uint32_t>> associations;
	map<uint32_t, pair<VectorXd, MatrixXd>> fusedTrackStates;
	//system track id->node track id->state
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> associatedStates; //states of node tarcks that have associated with system track according to associatio table 
	fusedTracks->statePrediction(fusionTime, parameters.largeProcessNoiseStd(), fusedTrackStates);
	fusedTracks->initializeAssociatedState(associatedStates);
	map<uint32_t, vector<uint32_t>> associatedNodeTrackIDs;
	for (uint32_t& id : subgroupIDs)
		associatedNodeTrackIDs.emplace(pair<uint32_t, vector<uint32_t>>(id, vector<uint32_t>()));
	fusedTracks->getInfoFromAssociationTable(parameters, fusionTime, lastHierarchyTrack, subgroupIDs, associations, associatedNodeTrackIDs, associatedStates);
	//get node track states
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> nodeTrackStates;
	for (uint32_t& subgroupID : subgroupIDs)
	{
		//cout << "node id = " << nodeID << ", num tracks = " << nodeTrack->second.getNumTracks() << endl;;
		vector<uint32_t> unassociatedIDs = lastHierarchyTrack[subgroupID].getUnassociatedTrackID(associatedNodeTrackIDs[subgroupID]);
		for (uint32_t& id : unassociatedIDs)
		{
			pair<VectorXd, MatrixXd> state = lastHierarchyTrack[subgroupID].getLatestTrackState(id);
			nodeTrackStates[subgroupID].emplace(pair<uint32_t, pair<VectorXd, MatrixXd>>(id, state));
		}
	}
	this->groupTrackFusion(parameters, fusionTime, hierarchy, group, fusedTracks, associations, fusedTrackStates, nodeTrackStates, associatedStates);
}

void HierarchicalRadarFusion::topHierarchyFusion(Parameters& parameters, double fusionTime, map<uint32_t, FusedTrack>& nodeTracks)
{
	FusedTrack* systemTracks = &(this->m_systemTrack);
	map<uint32_t, map<uint32_t, uint32_t>> associations;
	map<uint32_t, pair<VectorXd, MatrixXd>> fusedTrackStates;
	//system track id->node track id->state
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> associatedStates; //states of node tarcks that have associated with system track according to associatio table 
	systemTracks->statePrediction(fusionTime, parameters.largeProcessNoiseStd(), fusedTrackStates);
	systemTracks->initializeAssociatedState(associatedStates);
	map<uint32_t, vector<uint32_t>> associatedNodeTrackIDs;
	for (map<uint32_t, FusedTrack>::iterator iter = nodeTracks.begin(); iter != nodeTracks.end(); iter++)
		associatedNodeTrackIDs.emplace(pair<uint32_t, vector<uint32_t>>(iter->first, vector<uint32_t>()));
	//get tracks in the association table
	systemTracks->getInfoFromAssociationTable(parameters, fusionTime, nodeTracks, associations, associatedNodeTrackIDs, associatedStates);
	//get node track states
	map<uint32_t, map<uint32_t, pair<VectorXd, MatrixXd>>> nodeTrackStates;
	for (map<uint32_t, FusedTrack>::iterator nodeTrack = nodeTracks.begin(); nodeTrack != nodeTracks.end(); nodeTrack++)
	{
		uint32_t nodeID = nodeTrack->first;
		//cout << "node id = " << nodeID << ", num tracks = " << nodeTrack->second.getNumTracks() << endl;;
		vector<uint32_t> unassociatedIDs = nodeTrack->second.getUnassociatedTrackID(associatedNodeTrackIDs[nodeID]);
		for (uint32_t& id : unassociatedIDs)
		{
			pair<VectorXd, MatrixXd> state = nodeTrack->second.getLatestTrackState(id);
			nodeTrackStates[nodeID].emplace(pair<uint32_t, pair<VectorXd, MatrixXd>>(id, state));
		}
	}
	uint32_t hierarchy = 0;
	uint32_t group = 0;
	this->groupTrackFusion(parameters, fusionTime, hierarchy, group, systemTracks, associations, fusedTrackStates, nodeTrackStates, associatedStates);
}

void HierarchicalRadarFusion::hierarchicalFusion(Parameters& parameters, double fusionTime)
{
	bool useSystemTrack = false;
	map<uint32_t, map<uint32_t, FusedTrack>>::iterator hierarchicalFusedTrack = this->m_hierarchicalFusedTrack.begin();
	if (this->m_hierarchicalFusedTrack.size() == 1) //only one hierarchy
	{
		if (hierarchicalFusedTrack->second.size() == 1) //only one group, fused tarck as system track
		{
			useSystemTrack = true;
			this->rootHierarchyFusion(parameters, fusionTime, useSystemTrack);
			//this->m_systemTrack = this->m_hierarchicalFusedTrack.begin()->second.begin()->second;
		}
		else if (hierarchicalFusedTrack->second.size() > 1) //more than one group but less than numElements + 1, fuse the fused tarck in each group to get system track
		{
			this->rootHierarchyFusion(parameters, fusionTime, useSystemTrack);
			this->topHierarchyFusion(parameters, fusionTime, this->m_hierarchicalFusedTrack.begin()->second);
		}
	}
	else
	{
		this->rootHierarchyFusion(parameters, fusionTime, useSystemTrack);
		hierarchicalFusedTrack++;
		for (; hierarchicalFusedTrack != this->m_hierarchicalFusedTrack.end(); hierarchicalFusedTrack++)
		{
			uint32_t hierarchy = hierarchicalFusedTrack->first;
			vector<thread> threads;
			for (map<uint32_t, FusedTrack>::iterator fusedTracks = hierarchicalFusedTrack->second.begin(); 
				fusedTracks != hierarchicalFusedTrack->second.end(); fusedTracks++)
			{
				map<uint32_t, map<uint32_t, FusedTrack>>::iterator lastHierarchy = prev(hierarchicalFusedTrack, 1);
				uint32_t groupID = fusedTracks->first;
				vector<uint32_t> subgroupIDs = this->m_subheirarchyIDs[hierarchy][groupID];
				thread threadMiddleHierarchyFusion(&HierarchicalRadarFusion::middleHierarchyFusion, this, ref(parameters), fusionTime,
					hierarchy, fusedTracks, ref(lastHierarchy->second), subgroupIDs);
				//this->middleHierarchyFusion(parameters, fusionTime, hierarchy, fusedTracks, lastHierarchy->second, subgroupIDs);
				threads.emplace_back(move(threadMiddleHierarchyFusion));
			}
			for (thread& t : threads)
				t.join();
		}
		advance(hierarchicalFusedTrack, -1);
		this->topHierarchyFusion(parameters, fusionTime, hierarchicalFusedTrack->second);
	}
}

void HierarchicalRadarFusion::getAssociationHistory(map<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>>& history)
{
	/*map<uint32_t, map<uint32_t, FusedTrack>>::iterator id = this->m_hierarchicalFusedTrack.begin();
	uint32_t baseHierarchyID = id->first;
	for (map<uint32_t, FusedTrack>::iterator iter = this->m_hierarchicalFusedTrack[baseHierarchyID].begin(); iter != this->m_hierarchicalFusedTrack[baseHierarchyID].end(); iter++)
	{
		map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>> associations;
		iter->second.getAssociationHistory(associations);
		history.emplace(pair<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>>(iter->first, associations));
	}*/
	map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>> associations;
	this->m_systemTrack.getAssociationHistory(associations);
	history.emplace(pair<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>>(0, associations));
}

void HierarchicalRadarFusion::displayAssociationHistory(map<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>>& history)
{
	map<uint32_t, map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>>::iterator group = history.begin();
	for (; group != history.end(); group++)
	{
		cout << "**** group = " << group->first  << endl;
		map<uint32_t, map<uint32_t, map<uint32_t, uint32_t>>>::iterator fusedTrack = group->second.begin();
		for (; fusedTrack != group->second.end(); fusedTrack++)
		{
			cout << "*** fused track id = " << fusedTrack->first << ", associated with: " << endl;
			for (map<uint32_t, map<uint32_t, uint32_t>>::iterator group = fusedTrack->second.begin(); group != fusedTrack->second.end(); group++)
			{
				cout << "** node id = " << group->first << endl;
				for (map<uint32_t, uint32_t>::iterator nodeTrack = group->second.begin(); nodeTrack != group->second.end(); nodeTrack++)
				{
					cout << "* node track id = " << nodeTrack->first << ", number of association = " << nodeTrack->second << endl;
				}
			}
		}
		cout << "------------------------------------------------" << endl;
	}
}

void HierarchicalRadarFusion::plotFilterdRadarTrack(string coordinate, string plotOrWrite)
{
	int index = 0;
	for (map<uint32_t, map<uint32_t, RadarTrack>>::iterator iterGroup = this->m_radarTrack.begin(); iterGroup != this->m_radarTrack.end(); iterGroup++)
	{
		for (map<uint32_t, RadarTrack>::iterator iterRadar = iterGroup->second.begin(); iterRadar != iterGroup->second.end(); iterRadar++)
		{
			map<uint32_t, vector<VectorXd>> tracks;
			iterRadar->second.getTracks(tracks);
			if (plotOrWrite.compare("plot") == 0)
			{
				if (!tracks.empty())
				{
					OpencvPlot::plotTracks(tracks, coordinate);
				}
			}
			else if (plotOrWrite.compare("write") == 0)
			{
				index++;
				fstream ofs;
				string fileName = "filteredTracks\\" + to_string(index) + ".txt";
				ofs.open(fileName, ios::out);
				for (map<uint32_t, vector<VectorXd>>::iterator iter = tracks.begin(); iter != tracks.end(); iter++)
				{
					vector<vector<double>> position(3);
					for (VectorXd& state : iter->second)
					{
						Vector3d location = CoordinateTransition::ecef2geo(state(0), state(3), state(6), wgs84);
						position[0].emplace_back(location(0) / D2R);
						position[1].emplace_back(location(1) / D2R);
						position[2].emplace_back(location(2));
					}
					for (size_t i = 0; i < position.size(); i++)
					{
						for (size_t j = 0; j < position[i].size(); j++)
							ofs << position[i][j] << '\t';
						ofs << endl;
					}
				}
				ofs.close();
			}
		}
	}
}

void HierarchicalRadarFusion::plotSystemTrack(string coordinate, string plotOrWrite)
{
	map<uint32_t, vector<VectorXd>> systemTracks = this->m_systemTrack.getTracks();
	if (plotOrWrite.compare("plot") == 0)
	{
		if (!systemTracks.empty())
		{
			//OpencvPlot::plotTracks(systemTracks, coordinate);
			MatPlot::plotTracks(systemTracks, coordinate);
		}
	}
}