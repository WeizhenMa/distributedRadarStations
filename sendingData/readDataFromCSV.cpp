#include "readDataFromCSV.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <WS2tcpip.h>
#include <windows.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#pragma comment (lib, "ws2_32.lib")  
#pragma warning(disable : 4996)

using namespace std;
using namespace Json;

NewtworkConfiguration::NewtworkConfiguration()
{
	this->m_portServer = 9001;
	this->m_ipServer = "127.0.0.1";
	this->m_portSendingClient = 10000;
}

int NewtworkConfiguration::getServerPort()
{
	return this->m_portServer;
}

string NewtworkConfiguration::getServerIp()
{
	return this->m_ipServer;
}

int NewtworkConfiguration::getClientPort()
{
	return this->m_portSendingClient;
}

TrackInfo::TrackInfo()
{
	this->m_detectedID = 0;
	this->m_detectedTime = 0;
	this->m_entityID = 0;
	this->m_height = 0;
	this->m_lla[0] = 0;
	this->m_lla[1] = 0;
	this->m_lla[2] = 0;
	this->m_radarID = 0;
	this->m_targetType = 0;
	this->m_trueID = 0;
	this->m_velocity[0] = 0;
	this->m_velocity[1] = 0;
	this->m_velocity[2] = 0;
}

void TrackInfo::writeTrackData(istringstream istr)
{
	string str;
	getline(istr, str, ',');
	double simTime = atof(str.c_str());
	getline(istr, str, ',');
	this->m_detectedTime = atof(str.c_str());
	getline(istr, str, ',');
	this->m_detectedID = atoi(str.c_str());
	getline(istr, str, ',');
	this->m_trueID = atoi(str.c_str());
	getline(istr, str, ','); 
	this->m_radarID = atoi(str.c_str());
	getline(istr, str, ','); 
	this->m_entityID = atoi(str.c_str());
	getline(istr, str, ','); 
	this->m_targetType = atoi(str.c_str());
	getline(istr, str, ','); 
	this->m_lla[0] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_lla[1] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_lla[2] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_velocity[0] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_velocity[1] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_velocity[2] = atof(str.c_str());
	getline(istr, str, ','); 
	this->m_height = atof(str.c_str());
	//this->m_lla[2] = this->m_height;
}

int TrackInfo::getRadarID()
{
	return this->m_radarID;
}

void TrackInfo::writeData2Json(Value& root)
{
	root["detectedTime"] = this->m_detectedTime;
	root["detectedID"] = this->m_detectedID;
	root["trueID"] = this->m_trueID;
	root["entityID"] = this->m_entityID;
	root["radarID"] = this->m_radarID;
	root["targetType"] = this->m_targetType;
	root["lla"].append(this->m_lla[0]); 
	root["lla"].append(this->m_lla[1]); 
	root["lla"].append(this->m_lla[2]);
	root["velocity"].append(this->m_velocity[0]); 
	root["velocity"].append(this->m_velocity[1]); 
	root["velocity"].append(this->m_velocity[2]);
	root["height"] = this->m_height;
}

RadarData::RadarData()
{
	this->m_id = 0;
}

void RadarData::writeRadarID(int id)
{
	this->m_id = id;
}

int RadarData::readRadarID()
{
	return this->m_id;
}

void RadarData::writeNewTrack(TrackInfo track)
{
	this->m_track.emplace_back(track);
}

void RadarData::writeData2Json(Value& roots)
{
	for (TrackInfo& track : this->m_track)
	{
		Value root;
		track.writeData2Json(root);
		roots["tracks"].append(root);
	}
}

void CSVData::readData(string fileName)
{
	ifstream ifs;
	ifs.open(fileName, ios::in);
	if (!ifs.is_open())
	{
		cout << "cannot open the file to read CSV data!" << endl;
		return;
	}
	vector<string> stringData;
	string line;
	getline(ifs, line);
	while (getline(ifs, line)) stringData.emplace_back(line);
	map<int, RadarData> emptyData;
	double invalidTime = -1000;
	pair<double, map<int, RadarData>> singleStepData(invalidTime, emptyData);
	for (vector<string>::iterator iter = stringData.begin(); iter != stringData.end(); iter++)
	{
		string str;
		istringstream istr(*iter);
		getline(istr, str, ','); 
		double simTime = atof(str.c_str());
		getline(istr, str, ','); 
		double detectedTime = atof(str.c_str());
		TrackInfo newTrack;
		newTrack.writeTrackData(istringstream (*iter));
		int radarID = newTrack.getRadarID();
		if (detectedTime != singleStepData.first)
		{
			if (singleStepData.first != invalidTime) this->m_data.insert(singleStepData);
			singleStepData.first = detectedTime;
			singleStepData.second.clear();
			RadarData radarData;
			radarData.writeRadarID(radarID);
			radarData.writeNewTrack(newTrack);
			pair<int, RadarData> singleRadarData(radarID, radarData);
			singleStepData.second.insert(singleRadarData);
		}
		else
		{
			map<int, RadarData>::iterator iter = singleStepData.second.find(radarID);
			if (iter != singleStepData.second.end()) 
			{
				iter->second.writeNewTrack(newTrack);
			}
			else
			{
				RadarData radarData;
				radarData.writeRadarID(radarID);
				radarData.writeNewTrack(newTrack);
				pair<int, RadarData> singleRadarData(radarID, radarData);
				singleStepData.second.insert(singleRadarData);
			}
		}
	}
	this->m_data.insert(singleStepData);
	//for (map<double, map<int, RadarData>>::iterator iter = this->m_data.begin(); iter != this->m_data.end(); iter++)
	//{
	//	cout << "time stamp = " << iter->first << endl;
	//}
}

int CSVData::getDataSize()
{
	return this->m_data.size();
}

void CSVData::writeData2Json()
{
	size_t numSteps = this->m_data.size();
	int step = 0;
	for (map<double, map<int, RadarData>>::iterator iterStep = this->m_data.begin(); iterStep != this->m_data.end(); iterStep++)
	{
		Value singleStepData;
		for (map<int, RadarData>::iterator iterRadar = iterStep->second.begin(); iterRadar != iterStep->second.end(); iterRadar++)
		{
			Value singleRadarData;
			singleRadarData["radarID"] = iterRadar->first;
			iterRadar->second.writeData2Json(singleRadarData);
			singleStepData["data"].append(singleRadarData);
		}
		step++;
		if (step < numSteps) singleStepData["terminator"] = 0;
		else if (step == numSteps) singleStepData["terminator"] = 1;
		//cout << singleStepData << endl;
		string fileName = "jsonData/" + to_string(step) + ".json";
		ofstream ofs;
		ofs.open(fileName, ios::out);
		StyledWriter sw;
		ofs << sw.write(singleStepData);
		ofs.close();
	}
}

void CSVData::sendingData(NewtworkConfiguration network)
{
	size_t numSteps = this->m_data.size();
	size_t step = 0;
	for (map<double, map<int, RadarData>>::iterator iterStep = this->m_data.begin(); iterStep != this->m_data.end(); iterStep++)
	{
		Value singleStepData;
		for (map<int, RadarData>::iterator iterRadar = iterStep->second.begin(); iterRadar != iterStep->second.end(); iterRadar++)
		{
			Value singleRadarData;
			singleRadarData["radarID"] = iterRadar->first;
			iterRadar->second.writeData2Json(singleRadarData);
			singleStepData["data"].append(singleRadarData);
		}
		step++;
		if (step < numSteps) singleStepData["terminator"] = 0;
		else if (step == numSteps) singleStepData["terminator"] = 1;
		cout << singleStepData << endl;
		sendingData2Server(network, singleStepData);
		boost::asio::io_service io;
		boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(100));
		t.wait();
	}
}

void CSVData::sendingData2Server(NewtworkConfiguration network, Value root)
{
	int BUFF_SIZE = 10000;
	bool reuseAddress = true;
	SetConsoleOutputCP(CP_UTF8);
	Json::FastWriter fw;
	string radarData = fw.write(root);
	WSADATA data;
	WORD version = MAKEWORD(2, 2);
	int wsOk = WSAStartup(version, &data);
	if (wsOk != 0)
	{
		cout << "Can't start Winsock! " << wsOk;
		return;
	}
	sockaddr_in server;
	server.sin_family = AF_INET; // AF_INET = IPv4 addresses
	server.sin_port = htons(network.getServerPort()); // Little to big endian conversion
	server.sin_addr.S_un.S_addr = inet_addr(network.getServerIp().c_str()); //192.168.31.1     192.168.1.116
	// Socket creation, note that the socket type is datagram
	SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);
	setsockopt(out, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuseAddress, sizeof(bool));
	sockaddr_in addrLocal;
	memset(&addrLocal, 0, sizeof(addrLocal));
	addrLocal.sin_family = AF_INET;
	addrLocal.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrLocal.sin_port = htons(network.getClientPort());
	if (::bind(out, (sockaddr*)&addrLocal, sizeof(addrLocal)) == SOCKET_ERROR) //固定端口发送，不由系统随机分配
	{
		cout << "Can't bind socket in sendingData2Intention! " << WSAGetLastError() << endl;
		system("pause");
	}
	char* buff = new char[BUFF_SIZE];
	if (buff != NULL)
	{
		strcpy(buff, radarData.c_str());
		int sendOk = sendto(out, buff, strlen(buff), 0, (sockaddr*)&server, sizeof(server));
		cout << buff << endl;
	}
	delete []buff;
	closesocket(out);
	WSACleanup();
}