#pragma once
#include <vector>
#include <Eigen/Dense>
#include "json/json.h"
#include <map>
#include <queue>
#include <mutex>

#pragma comment(lib, "jsoncpp.lib")

class NewtworkConfiguration
{
private:
	uint16_t m_portReceivingData;
public:
	NewtworkConfiguration();
	uint16_t getReceivingPort();
};

class ReceivedRadarTrack
{
private:
	std::vector<uint32_t> m_radarID;
	std::vector<uint32_t> m_entityID;
	std::vector<double> m_timeStamps;
	std::vector<uint32_t> m_detectedID;
	std::vector<uint32_t> m_trueID;
	std::vector<uint32_t> m_targetType;
	std::vector<Eigen::Vector3d> m_position;
	std::vector<Eigen::Vector3d> m_velocity;
	std::vector<double> m_height;
public:
	void writeData(Json::Value root, const uint32_t trackIDFactor);
	void readRadarID(std::vector<uint32_t>& radarID);
	void readEntityID(std::vector<uint32_t>& entityID);
	void readTimeStamps(std::vector<double>& timeStamps);
	void readDetectedID(std::vector<uint32_t>& detectedID);
	void readTrueID(std::vector<uint32_t>& trueID);
	void readTargetType(std::vector<uint32_t>& targetType);
	void readPosition(std::vector<Eigen::Vector3d>& position);
	void readVelocity(std::vector<Eigen::Vector3d>& velocity);
	void readHeight(std::vector<double>& height);
};

class ReceivedData
{
private:
	std::mutex m_mutex;
	bool m_terminator;
	std::queue<std::map<uint32_t, ReceivedRadarTrack>> m_data;
	std::vector<double> m_timeStamps;
public:
	ReceivedData();
	void receivingData(NewtworkConfiguration network, const uint32_t trackIDFactor);
	void writeData(Json::Value root, const uint32_t trackIDFactor);
	void readData(std::queue<std::map<uint32_t, ReceivedRadarTrack>>& data);
	void writeDataFromJson(std::string fileFolder, double timeStamp, std::pair<int, int> stepIndices, const uint32_t trackIDFactor);
	void writeTimeStamps(std::string fileName);
	void getDataIndices(double simTime, std::pair<int, int>& stepIndices);
};