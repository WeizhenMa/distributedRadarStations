#pragma once
#include <vector>
#include <string>
#include <map>
#include "json/json.h"

#pragma comment(lib, "jsoncpp.lib")

class NewtworkConfiguration
{
private:
	int m_portServer;
	std::string m_ipServer;
	int m_portSendingClient;
public:
	NewtworkConfiguration();
	int getServerPort();
	std::string getServerIp();
	int getClientPort();
};

class TrackInfo
{
private:
	double m_detectedTime;
	int m_detectedID;
	int m_trueID;
	int m_radarID;
	int m_entityID;
	int m_targetType;
	double m_lla[3];
	double m_velocity[3];
	double m_height;
public:
	TrackInfo();
	int getRadarID();
	void writeTrackData(std::istringstream istr);
	void writeData2Json(Json::Value& root);
};

class RadarData
{
private:
	int m_id;
	std::vector<TrackInfo> m_track;
public:
	RadarData();
	void writeRadarID(int id);
	int readRadarID();
	void writeNewTrack(TrackInfo track);
	void writeData2Json(Json::Value& root);
};

class CSVData
{
private:
	std::map<double, std::map<int, RadarData>> m_data;
public:
	void readData(std::string fileName);
	int getDataSize();
	void writeData2Json();
	void sendingData(NewtworkConfiguration network);
	void sendingData2Server(NewtworkConfiguration network, Json::Value root);
};