#pragma once
#include "plot.h"
#include <CvPlot/cvplot.h>
#include <numeric>
#include "functions.h"
#include "coordinateTransition.h"
#include <matplot/matplot.h>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace matplot;

#define LineWith 1
#define MarkerSize 4
#define D2R 0.017453292519943

void OpencvPlot::plotTracks(map<uint32_t, vector<VectorXd>> states, string coordinate)
{
	WGS84 wgs84;
	vector<cv::Scalar> colors;
	for (size_t i = 0; i < 500; i++)
		colors.emplace_back(cv::Scalar(Functions::uniformRand(0, 255), Functions::uniformRand(0, 255), Functions::uniformRand(0, 255)));
	map<int, vector<Vector3d>>::iterator iter;
	auto axes = CvPlot::makePlotAxes();
	size_t index = 0;
	map<uint32_t, vector<vector<double>>> targetPosition;
	for (map<uint32_t, vector<VectorXd>>::iterator iter = states.begin(); iter != states.end(); iter++)
	{
		vector<vector<double>> position(3);
		if (coordinate.compare("lla") == 0)
		{
			for (VectorXd s : iter->second)
			{
				Vector3d location = CoordinateTransition::ecef2geo(s(0), s(3), s(6), wgs84);
				position[0].emplace_back(location(0) / D2R);
				position[1].emplace_back(location(1) / D2R);
				position[2].emplace_back(location(2));
			}
		}
		else if (coordinate.compare("ecef") == 0)
		{
			for (VectorXd s : iter->second)
			{
				position[0].emplace_back(s(0));
				position[1].emplace_back(s(3));
				position[2].emplace_back(s(6));
			}
		}
		targetPosition.insert(pair<int, vector<vector<double>>>(iter->first, position));
		axes.create<CvPlot::Series>(vector<double>{position[0].front()}, vector<double>{position[1].front()})
			.setColor(cv::Scalar(0, 255, 0))
			.setLineWidth(5)
			.setMarkerType(CvPlot::MarkerType::Circle)
			.setMarkerSize(MarkerSize);
		axes.create<CvPlot::Series>(vector<double>{position[0].back()}, vector<double>{position[1].back()})
			.setColor(cv::Scalar(0, 0, 255))
			.setLineWidth(5)
			.setMarkerType(CvPlot::MarkerType::Circle)
			.setMarkerSize(MarkerSize);
		axes.create<CvPlot::Series>(position[0], position[1], "-b")
			.setColor(colors[index])
			.setLineWidth(2)
			.setLineType(CvPlot::LineType::Solid)
			.setMarkerType(CvPlot::MarkerType::Circle)
			.setMarkerSize(4);
		index++;
	}
	if (coordinate.compare("lla") == 0)
	{
		axes.title("geo longitude - latitude track");
		axes.xLabel("longitude (degree)");
		axes.yLabel("latitude (degree)");
		CvPlot::show("geo longitude - latitude track", axes);
	}
	else if (coordinate.compare("ecef") == 0)
	{
		axes.title("ecef x - y track");
		axes.xLabel("x coordinate (m)");
		axes.yLabel("y coordinate (m)");
		CvPlot::show("ecef x - y track", axes);
	}
}

void MatPlot::plotTracks(map<uint32_t, vector<VectorXd>> states, string coordinate)
{
	WGS84 wgs84;
	vector<array<float, 4>> colors;
	for (size_t i = 0; i < 2000; i++)
		colors.emplace_back(to_array(vector<double>{ Functions::uniformRand(), Functions::uniformRand(), Functions::uniformRand() }));
	int markerSize = 15;
	size_t index = 0;
	for (map<uint32_t, vector<VectorXd>>::iterator iter = states.begin(); iter != states.end(); iter++)
	{
		vector<vector<double>> position(3);
		vector<vector<double>> lla(3);
		for (auto state : iter->second)
		{
			position[0].emplace_back(state(0));
			position[1].emplace_back(state(3));
			position[2].emplace_back(state(6));
			Vector3d location = CoordinateTransition::ecef2geo(state(0), state(3), state(6), wgs84);
			lla[0].emplace_back(location(0) / D2R);
			lla[1].emplace_back(location(1) / D2R);
			lla[2].emplace_back(location(2));
		}
		if (coordinate.compare("ecef") == 0)
		{
			auto line = plot3(position[0], position[1], position[2], "*");
			line->marker_color(colors[index]);
			line->marker_size(MarkerSize);
		}
		else if(coordinate.compare("lla") == 0)
		{
			auto line = plot3(lla[0], lla[1], lla[2], "*");
			line->marker_color(colors[index]);
			line->marker_size(MarkerSize);
		}
		hold(on);
		index++;
	}
	if (coordinate.compare("ecef") == 0)
	{
		xlabel("x coordinate (m)");
		ylabel("y coordinate (m)");
		zlabel("z coordinate (m)");
	}
	else if(coordinate.compare("lla") == 0)
	{
		xlabel("longitude (degree)");
		ylabel("latitude (degree)");
		zlabel("altitude (m)");
	}
	box(on);
	show();
}