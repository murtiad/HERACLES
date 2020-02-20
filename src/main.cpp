#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include "las2pcd.h"
#include "PARVIS.h"
#include "ROSACE.h"
#include "PIER.h"
#include "REGEMENT.h"
#include "CORDON.h"
#include "normal.h"

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
	cout << "regement	-- region growing segmentation" << endl;
	cout << "cordon	-- finds correspondance between two overlapping point clouds" << endl;
	cout << "keyston	-- detects 3D SIFT keypoints" << endl;
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
	else if (choice == "regement")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		regement(filePath);
	}
	else if (choice == "cordon")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path of the first point cloud: (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		string filePath2;

		cout << "Enter full .las or .pcd file path of the second point cloud: (or you can also drag the file here)" << endl;
		getline(cin, filePath2);

		cordon(filePath,filePath2);
	}
	else if (choice == "keyston")
	{
		cout << endl;
		cout << "Enter full .las or .pcd file path : (or you can also drag the file here)" << endl;
		getline(cin, filePath);

		cerr << "Loading " << filePath << endl;

		string file_extension1 = boost::filesystem::extension(filePath);

		PointCloud<PointXYZRGB>::Ptr input1(new PointCloud<PointXYZRGB>);

		if (file_extension1 == ".las")
		{
			*input1 = las2pcd(filePath);
		}
		else if (file_extension1 == ".pcd")
		{
			pcl::io::loadPCDFile(filePath, *input1);
		}
		else
		{
			cout << "Sorry! This file extension is not supported :(" << endl;
		}

		cout << "Successfully loaded " << filePath << endl;
		cout << endl;

		cout << "======================================================" << endl;
		cout << "KEYSTON (KEYpoints Simple deTectiON)" << endl;
		cout << "Detects keypoints in a point cloud" << endl;
		cout << "ver 0.1 - 8 March 2018" << endl;
		cout << "(c) Arnadi Murtiyoso" << endl;
		cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
		cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
		cout << "https://github.com/murtiad" << endl;
		cout << "======================================================" << endl;
		cout << endl;

		cerr << "Computing normals for both point clouds..." << endl;
		PointCloud<Normal>::Ptr cloud_normals1(new PointCloud<Normal>);

		cloud_normals1 = normal(input1, 0.015);
		cerr << "Normals computed!" << endl;
		cerr << endl;

		PointCloud<PointXYZ>::Ptr keypoints1(new PointCloud<PointXYZ>);

		keypoints1 = keyston(input1, cloud_normals1);
		cerr << "Detected " << keypoints1->points.size() << " 3D SIFT keypoints in the first point cloud!" << endl;
		cerr << endl;

		io::savePCDFileASCII("keyston_o.pcd", *keypoints1);
		cerr << "Saved " << keypoints1->points.size() << " points to cordon_o_keypoints1.pcd." << endl;
		cout << endl;
	}
	else if (choice == "exit")
	{
		
	}
	else
	{
		cout << "Command not recognized!" << endl;
	}
	//
	//string filePath;

	//std::cerr << "Loading " << filePath << std::endl;

	//string file_extension = boost::filesystem::extension(filePath);

	//PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	//if (file_extension == ".las")
	//{
	//	*cloud = las2pcd(filePath);
	//}
	//else if (file_extension == ".pcd")
	//{
	//	pcl::io::loadPCDFile(filePath, *cloud);
	//}
	//else
	//{
	//	cout << "Sorry! This file extension is not supported :(" << endl;
	//}

	//cout << "Successfully loaded " << filePath << endl;


	////Normal estimation
	//// Create the normal estimation class, and pass the input dataset to it
	//NormalEstimation<PointXYZRGB, PointNormal> ne;
	//ne.setInputCloud(cloud);

	//// Create an empty kdtree representation, and pass it to the normal estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	//ne.setSearchMethod(tree);

	//// Output datasets
	//PointCloud<PointNormal>::Ptr cloud_normals(new PointCloud<PointNormal>);

	//// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch(0.03);

	//// Compute the features
	//ne.compute(*cloud_normals);

	//// SIFT

	//SIFTKeypoint<PointNormal, PointXYZRGB> sift;
	//PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>);
	//sift.setInputCloud(cloud_normals);
	//sift.compute(*output);

	//// VISUALISATION

	//pcl::visualization::CloudViewer viewer("PARVIS v0.1");

	////blocks until the cloud is actually rendered
	//viewer.showCloud(cloud);

	//cout << endl;
	//cout << "Successfully loaded " << filePath << endl;

	////use the following functions to get access to the underlying more advanced/powerful
	////PCLVisualizer

	////This will only get called once
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	////This will get called once per visualization iteration
	//viewer.runOnVisualizationThread(viewerPsycho);
	//while (!viewer.wasStopped())
	//{
	//	//you can also do cool processing here
	//	//FIXME: Note that this is running in a separate thread from viewerPsycho
	//	//and you should guard against race conditions yourself...
	//	user_data++;
	//}


	// "Press enter to quit" code
	cin.ignore(std::cin.rdbuf()->in_avail());
    cout << endl << "Buh-bye! Press enter to terminate program." << endl;
    cin.ignore();
	return 0;
}