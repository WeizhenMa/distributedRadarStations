#include "receivingData.h"
#include <WS2tcpip.h>
#include <windows.h>
#include <iostream>
#include "coordinateTransition.h"
#include <fstream>

using namespace std;
using namespace Json;
using namespace Eigen;

#pragma comment (lib, "ws2_32.lib")  
#pragma warning(disable : 4996)

#ifndef D2R
#define D2R 0.017453292519943
#endif 

#ifndef wgs84
#define wgs84 WGS84()
#endif 

NewtworkConfiguration::NewtworkConfiguration()
{
	this->m_portReceivingData = 9001;
}

uint16_t NewtworkConfiguration::getReceivingPort()
{
	return this->m_portReceivingData;
}

void ReceivedRadarTrack::writeData(Value root, const uint32_t trackIDFactor)
{
	for (int t = 0; t < root.size(); t++)
	{
		uint32_t radarID = root[t]["radarID"].asUInt();
		uint32_t detectedID = root[t]["detectedID"].asUInt();
		this->m_detectedID.emplace_back(radarID * trackIDFactor + detectedID);
		this->m_entityID.emplace_back(root[t]["entityID"].asUInt());
		this->m_height.emplace_back(root[t]["height"].asDouble());
		double longitude = root[t]["lla"][0].asDouble() * D2R;
		double latitude = root[t]["lla"][1].asDouble() * D2R;
		double altitude = root[t]["lla"][2].asDouble();
		Vector3d position = CoordinateTransition::geo2ecef(longitude, latitude, altitude, wgs84);
		this->m_position.emplace_back(position);
		this->m_radarID.emplace_back(radarID);
		this->m_targetType.emplace_back(root[t]["targetType"].asUInt());
		this->m_timeStamps.emplace_back(root[t]["detectedTime"].asDouble());
		this->m_trueID.emplace_back(root[t]["trueID"].asUInt());
		this->m_velocity.emplace_back(Vector3d{ root[t]["velocity"][0].asDouble(), 
			root[t]["velocity"][1].asDouble(), root[t]["velocity"][2].asDouble() });
	}
}

void ReceivedRadarTrack::readRadarID(vector<uint32_t>& radarID)
{
	radarID.assign(this->m_radarID.begin(), this->m_radarID.end());
}

void ReceivedRadarTrack::readEntityID(vector<uint32_t>& entityID)
{
	entityID.assign(this->m_entityID.begin(), this->m_entityID.end());
}

void ReceivedRadarTrack::readTimeStamps(vector<double>& timeStamps)
{
	timeStamps.assign(this->m_timeStamps.begin(), this->m_timeStamps.end());
}

void ReceivedRadarTrack::readDetectedID(vector<uint32_t>& detectedID)
{
	detectedID.assign(this->m_detectedID.begin(), this->m_detectedID.end());
}

void ReceivedRadarTrack::readTrueID(vector<uint32_t>& trueID)
{
	trueID.assign(this->m_trueID.begin(), this->m_trueID.end());
}

void ReceivedRadarTrack::readTargetType(vector<uint32_t>& targetType)
{
	targetType.assign(this->m_targetType.begin(), this->m_targetType.end());
}

void ReceivedRadarTrack::readPosition(vector<Vector3d>& position)
{
	position.assign(this->m_position.begin(), this->m_position.end());
}

void ReceivedRadarTrack::readVelocity(vector<Vector3d>& velocity)
{
	velocity.assign(this->m_velocity.begin(), this->m_velocity.end());
}

void ReceivedRadarTrack::readHeight(vector<double>& height)
{
	height.assign(this->m_height.begin(), this->m_height.end());
}

ReceivedData::ReceivedData()
{
	this->m_terminator = false;
}

void ReceivedData::receivingData(NewtworkConfiguration network, const uint32_t trackIDFactor)
{
	bool reuseAddress = true;
	int BUFF_SIZE = 10000;
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int tmp = WSAStartup(version, &data);
	if (tmp != 0)
	{
		cout << "WSAStartup error" << endl;
	}
	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(in, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddress, sizeof(bool));
	sockaddr_in server;
	server.sin_addr.S_un.S_addr = ADDR_ANY; // Us any IP address available on the machine
	server.sin_family = AF_INET; // Address format is IPv4
	server.sin_port = htons(network.getReceivingPort()); // Convert from little to big endian
	if (::bind(in, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
	{
		cout << "Can't bind socket in recieving sensor data! " << WSAGetLastError() << endl;
		system("pause");
	}
	sockaddr_in client; // Use to hold the client information (port / ip address)
	int clientLength = sizeof(client); // The size of the client information
	ZeroMemory(&client, clientLength); // Clear the client structure
	char* buff = new char[BUFF_SIZE];
	while (!this->m_terminator)
	{
		int bytesIn = recvfrom(in, buff, BUFF_SIZE, 0, (sockaddr*)&client, &clientLength);
		if (bytesIn == SOCKET_ERROR)
		{
			cout << "Error receiving from client " << WSAGetLastError() << endl;
			continue;
		}
		//char clientIp[256]; // Create enough space to convert the address byte array
		//ZeroMemory(clientIp, 256); // to string of characters
		//inet_ntop(AF_INET, &client.sin_addr, clientIp, 256); //get client ip
		Value root;
		Reader reader;
		if (reader.parse(buff, root))
		{
			//cout << root << endl;
			this->writeData(root, trackIDFactor);
		}
	}
	closesocket(in);
	WSACleanup();
}

void ReceivedData::writeData(Value root, const uint32_t trackIDFactor)
{
	this->m_terminator = root["terminator"].asBool();
	map<uint32_t, ReceivedRadarTrack> multipleRadarData;
	for (int r = 0; r < root["data"].size(); r++)
	{
		uint32_t radarID = root["data"][r]["radarID"].asUInt();
		ReceivedRadarTrack radarTracks;
		radarTracks.writeData(root["data"][r]["tracks"], trackIDFactor);
		pair<uint32_t, ReceivedRadarTrack> singleRadarData(radarID, radarTracks);
		multipleRadarData.insert(singleRadarData);
	}
	lock_guard<mutex> lockGuard(this->m_mutex);
	this->m_data.push(multipleRadarData);
}

void ReceivedData::readData(queue<map<uint32_t, ReceivedRadarTrack>>& data)
{
	lock_guard<mutex> lockGuard(this->m_mutex);
	while (!this->m_data.empty())
	{
		data.push(this->m_data.front());
		this->m_data.pop();
	}
}

void ReceivedData::writeDataFromJson(string fileFolder, double timeStamp, pair<int, int> stepIndices, const uint32_t trackIDFactor)
{
	if (stepIndices.second > 0)
	{
		for (int step = 0; step < stepIndices.second; step++)
		{
			ifstream ifs;
			string fileName = fileFolder + to_string(stepIndices.first + 1 + step) + ".json";
			ifs.open(fileName, ios::in);
			Value root;
			Reader reader;
			if (reader.parse(ifs, root))
			{
				//cout << root << endl;
				this->writeData(root, trackIDFactor);
			}
			ifs.close();
		}
	}
}

void ReceivedData::writeTimeStamps(string fileName)
{
	ifstream ifs;
	ifs.open(fileName, ios::in);
	string line;
	while (getline(ifs, line)) this->m_timeStamps.emplace_back(atof(line.c_str()));
	ifs.close();
}

void ReceivedData::getDataIndices(double simTime, pair<int, int>& stepIndices)
{
	int startStep = stepIndices.first;
	if (stepIndices.second > 0)
	{
		startStep = stepIndices.first + stepIndices.second;
		stepIndices.first = startStep;
	}
	int counter = 0;
	while (true)
	{
		double timeStamp = this->m_timeStamps[startStep];
		startStep++;
		if (timeStamp <= simTime) counter++;
		else break;
	}
	stepIndices.second = counter;
}