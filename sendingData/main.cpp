#include <iostream>
#include "readDataFromCSV.h"
#include <fstream>

using namespace std;

int main()
{
	NewtworkConfiguration network;
	string filName = "data.csv";
	CSVData data;
	data.readData(filName);
	//data.sendingData(network);
	data.writeData2Json();
	//cout << data.getDataSize() << endl;
	system("pause");
	return 0;
}