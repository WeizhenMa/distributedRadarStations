#include "CoordinateTransition.h"
using namespace Eigen;

//ecef 地固坐标系（地心坐标系），笛卡尔，未反应与地球关系--xyz
//geo  大地坐标系（椭球、参心坐标系），参考椭球--经纬高
//enu  东北天坐标系（站心坐标系），用于局部的导航和定位

WGS84::WGS84()
{
	this->m_longSemiAxis = 6378137;
	this->m_shortSemiAxis = 6356752.3142;
	this->m_e1Square = 0.00669437999014;
	this->m_e2Square = 0.00673949674228;
}

double WGS84::longSemiAxis()
{
	return this->m_longSemiAxis;
}
double WGS84::shortSemiAxis()
{
	return this->m_shortSemiAxis;
}

double WGS84::e1Square()
{
	return this->m_e1Square;
}

double WGS84::e2Square()
{
	return this->m_e2Square;
}

//LLA to ECEF
Vector3d CoordinateTransition::geo2ecef(double longitude, double latitude, double altitude, WGS84 wgs84)
{
	Vector3d position;
	double constant = wgs84.longSemiAxis() / (pow(1 - wgs84.e1Square() * pow(sin(latitude), 2), 0.5));
	position(0) = (constant + altitude) * cos(latitude) * cos(longitude);
	position(1) = (constant + altitude) * cos(latitude) * sin(longitude);
	position(2) = (constant * (1 - wgs84.e1Square()) + altitude) * sin(latitude);
	return position;
}

//enu to ECEF
Vector3d CoordinateTransition::venu2vecef(double ve, double vn, double vu, double longitude, double latitude)
{
	Vector3d Venu(ve, vn, vu);
	Matrix3d transitionMatrix;
	transitionMatrix << -sin(longitude), -sin(latitude) * cos(longitude), cos(latitude)* cos(longitude),
		cos(longitude), -sin(latitude) * sin(longitude), cos(latitude)* sin(longitude),
		0, cos(latitude), sin(latitude);
	Vector3d Vecef = transitionMatrix * Venu;
	return Vecef;
}

//ECEF to enu position
Vector3d CoordinateTransition::ecef2enu(double xe, double ye, double ze, double longitude, double latitude, double altitude, WGS84 wgs84)
{
	Vector3d Xe(xe, ye, ze);
	Vector3d Xs = CoordinateTransition::geo2ecef(longitude, latitude, altitude, wgs84);
	Matrix3d transitionMatrix;
	transitionMatrix << -sin(longitude), -sin(latitude) * cos(longitude), cos(altitude)* cos(longitude),
		cos(longitude), -sin(latitude) * sin(longitude), cos(latitude)* sin(longitude),
		0, cos(latitude), sin(latitude);
	Vector3d Xn = transitionMatrix.transpose() * (Xe - Xs);
	return Xn;
}

//ECEF to enu velocity
Vector3d CoordinateTransition::vecef2venu(double xe, double ye, double ze, double longitude, double latitude)
{
	Vector3d Xe(xe, ye, ze);
	Matrix3d transitionMatrix;
	transitionMatrix << -sin(longitude), -sin(latitude) * cos(longitude), cos(latitude)* cos(longitude),
		cos(longitude), -sin(latitude) * sin(longitude), cos(latitude)* sin(longitude),
		0, cos(latitude), sin(latitude);
	Vector3d Xn = transitionMatrix.transpose() * Xe;
	return Xn;
}

// enu to ecef
Vector3d CoordinateTransition::enu2ecef(double xn, double yn, double zn, double longitude, double latitude, double altitude, WGS84 wgs84)
{
	Vector3d Xn(xn, yn, zn);
	Vector3d Xs = CoordinateTransition::geo2ecef(longitude, latitude, altitude, wgs84);
	Matrix3d transitionMatrix;
	transitionMatrix << -sin(longitude), -sin(latitude) * cos(longitude), cos(latitude)* cos(longitude),
		cos(longitude), -sin(latitude) * sin(longitude), cos(latitude)* sin(longitude),
		0, cos(latitude), sin(latitude);
	Vector3d Xe = Xs + transitionMatrix * Xn;
	return Xe;
}

//ECEF to LLA
Vector3d CoordinateTransition::ecef2geo(double x, double y, double z, WGS84 wgs84)
{
	double longitude = acos(x / pow(pow(x, 2) + pow(y, 2), 0.5));
	int numIterations = 10;
	double latitude = atan(z / pow(pow(x, 2) + pow(y, 2), 0.5));
	double constant = wgs84.longSemiAxis() / pow((1 - wgs84.e1Square() * pow(sin(latitude), 2)), 0.5);
	for (int i = 0; i < numIterations; i++)
	{
		latitude = atan((z + constant * wgs84.e1Square() * sin(latitude)) / pow(pow(x, 2) + pow(y, 2), 0.5));
		constant = wgs84.longSemiAxis() / pow((1 - wgs84.e1Square() * pow(sin(latitude), 2)), 0.5);
	}
	constant = wgs84.longSemiAxis() / pow((1 - wgs84.e1Square() * pow(sin(latitude), 2)), 0.5);
	double altitude = pow(pow(x, 2) + pow(y, 2), 0.5) / cos(latitude) - constant;
	Vector3d lla(longitude, latitude, altitude);
	return lla;
}