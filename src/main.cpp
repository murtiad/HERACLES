#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include "las2pcd.h"
#include "PARVIS.h"
#include "ROSACE.h"
#include "PIER.h"

using namespace std;
using namespace pcl;

int main (int argc, char** argv)
{
	cout << "======================================================" << endl;
	cout << "HERACLES (HERitAge by point CLoud procESsing)" << endl;
	cout << "ver 0.1 - 30 May 2017" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "======================================================" << endl;
	cout << endl;

	string choice;
	string filePath;

	cout << "Welcome! What would you like to do?" << endl;
	cout << "las2pcd	-- Converts .las point clouds into PCL-friendly format .pcd" << endl;
	cout << "parvis	-- Simple .las or .pcd point cloud visualizer" << endl;
	cout << "pier	-- Poisson-based surface reconstruction" << endl;
	cout << "rosace	-- RANSAC-based plane segmentation" << endl;
	cout << "exit	-- Exits the program" << endl;
	cout << "Your choice : ";
	getline(cin, choice);

	if (choice == "las2pcd")
	{
		PointCloud<PointXYZRGB> cloud;

		cout << endl;
		cout << "Enter full path for your .las file: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		cout << "Converting...Please be patient...";
		cloud = las2pcd(filePath);

		io::savePCDFileASCII ("pointcloud.pcd", cloud);
		cout << endl;
		cerr << "Saved " << cloud.points.size () << " data points to pointcloud.pcd." << endl;
	}
	else if (choice == "parvis")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		parvis(filePath);
	}
	else if (choice == "rosace")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		rosace(filePath);
	}
	else if (choice == "pier")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		pier(filePath);
	}
	else if (choice == "exit")
	{
		
	}
	else
	{
		cout << "Command not recognized!" << endl;
	}

	// "Press enter to quit" code
	cin.ignore(std::cin.rdbuf()->in_avail());
    cout << endl << "Buh-bye! Press enter to terminate program." << endl;
    cin.ignore();
	return 0;
}
