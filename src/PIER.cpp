#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/vtk_io.h>
#include <boost/filesystem.hpp>
#include "las2pcd.h"

using namespace std;
using namespace pcl;

////////////////////////////////////////////////////////////////////////
// PIER - Performs Poisson reconstruction on a point cloud
// ver 0.1 - 1 June 2017 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : Full file path of the .pcd file
// Output : Meshed 3D model in .vtk format
////////////////////////////////////////////////////////////////////////

int pier (string filePath)
{
	cout << "======================================================" << endl;
	cout << "PIER (PoIsson surfacE Reconstruction)" << endl;
	cout << "Performs Poisson reconstruction on a point cloud" << endl;
	cout << "ver 0.1 - 1 June 2017" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "======================================================" << endl;
	cout << endl;

	cerr << "Loading " << filePath << endl;

	string file_extension  = boost::filesystem::extension(filePath);

	PointCloud<PointXYZRGB>::Ptr input (new PointCloud<PointXYZRGB>);
	
	if (file_extension == ".las")
	{
		*input = las2pcd(filePath);
	}
	else if (file_extension == ".pcd")
	{
		pcl::io::loadPCDFile (filePath, *input);
	}
	else
	{
		cout << "Sorry! This file extension is not supported :(" << endl;
	}  

	cout << "Successfully loaded " << filePath << endl;

	// Suppresses the RGB for normal computation
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	pcl::copyPointCloud(*input,*cloud);

	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads (24);
	ne.setInputCloud (cloud);

	// Use all neighbors in a sphere of radius 
	ne.setRadiusSearch (0.1);

	// Calculate the centroid of the point cloud and put it in a vector
	cout << "Calculating centroid..."<< endl;
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud, centroid);
	cerr << "Centroid located at: " << centroid[0] << " " 
	                                    << centroid[1] << " "
	                                    << centroid[2] << endl;

	// Set the normal computation viewpoint to the centroid of the point cloud 
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	// Declare an object to hold the normal values
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());

	// Compute the normals
	cout << "Computing cloud normals..."<< endl;
	ne.compute (*cloud_normals);
	cout << "Normals computed!"<< endl;

	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////Alternate approach to calculate normals using KdTree//////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//// Create the normal estimation class, and pass the input dataset to it
	//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	//ne.setInputCloud (cloud);

	//// Create an empty kdtree representation, and pass it to the normal estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	//ne.setSearchMethod (tree);

	//// Output datasets
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	//// Use all neighbors in a sphere of radius 
	//ne.setRadiusSearch (0.4);

	//// Compute the features
	//ne.compute (*cloud_normals);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals
	
	// Creates the object to hold the resulting mesh
	PolygonMesh mesh;

	// Create the object from the class Poisson
	Poisson<PointNormal> poisson;

	// Sets the octree depth of the surface reconstruction
	poisson.setDepth (12);

	// Sets the input point cloud
	poisson.setInputCloud (cloud_with_normals);
	
	// Performs the meshing
	cout << "Meshing..." << endl;
	poisson.reconstruct (mesh);
	cout << "Meshing succesful." << endl;

	// Save to a .vtk file
	io::saveVTKFile ("PIER_mesh.vtk", mesh);
	cout << "Saved mesh to pier_o.vtk." << endl;

	// "Press enter to quit" code
	cin.ignore(std::cin.rdbuf()->in_avail());
    cout << endl << "Buh-bye! Press enter to terminate program." << endl;
    cin.ignore();
	return 0;
}