#pragma once
#include <cmath>
#include <Eigen/Dense>

class WGS84
{
private:
	double m_longSemiAxis;//长半径
	double m_shortSemiAxis;//短半径
	double m_e1Square; //第一偏心率的平方
	double m_e2Square;//第二偏心率的平方
public:
	WGS84();
	double longSemiAxis();
	double shortSemiAxis();
	double e1Square();
	double e2Square();
};

class CoordinateTransition
{
public:
	static Eigen::Vector3d geo2ecef(double lon, double lat, double alt, WGS84 wgs84);
	static Eigen::Vector3d venu2vecef(double ve, double vn, double vu, double lon, double lat);
	static Eigen::Vector3d ecef2enu(double xe, double ye, double ze, double lon, double lat, double alt, WGS84 wgs84);
	static Eigen::Vector3d vecef2venu(double xe, double ye, double ze, double lon, double lat);
	static Eigen::Vector3d enu2ecef(double xn, double yn, double zn, double lon, double lat, double alt, WGS84 wgs84);
	static Eigen::Vector3d ecef2geo(double x, double y, double z, WGS84 wgs84);
};