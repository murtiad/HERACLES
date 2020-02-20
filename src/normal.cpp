#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/vtk_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <boost/filesystem.hpp>
#include "las2pcd.h"

using namespace std;
using namespace pcl;

////////////////////////////////////////////////////////////////////////
// normal.cpp - Calculates point cloud normal
// ver 0.1 - 8 March 2018 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : PointCloud type XYZRGB and neighbor search radius
// Output : PointCloud type Normal
////////////////////////////////////////////////////////////////////////

PointCloud<Normal>::Ptr normal (PointCloud<PointXYZRGB>::Ptr input, float radiusSearch)
{
	//cout << "======================================================" << endl;
	//cout << "normal.cpp" << endl;
	//cout << "Calculates point cloud normal" << endl;
	//cout << "ver 0.1 - 8 March 2018" << endl;
	//cout << "(c) Arnadi Murtiyoso" << endl;
	//cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	//cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	//cout << "https://github.com/murtiad" << endl;
	//cout << "======================================================" << endl;
	//cout << endl;

	// Suppresses the RGB for normal computation
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	pcl::copyPointCloud(*input, *cloud);

	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads(24);
	ne.setInputCloud(cloud);

	// Use all neighbors in a sphere of radius 
	// IMPORTANT: the radius used here has to be smaller than the radius used to detect keypoints!!!
	ne.setRadiusSearch(radiusSearch);

	// Calculate the centroid of the point cloud and put it in a vector
	/*cout << "Calculating centroid..." << endl;*/
	Eigen::Vector4f centroid;
	compute3DCentroid(*cloud, centroid);
	/*cerr << "Centroid located at: " << centroid[0] << " "
		<< centroid[1] << " "
		<< centroid[2] << endl;*/

	// Set the normal computation viewpoint to the centroid of the point cloud 
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

	// Declare an object to hold the normal values
	PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());

	// Compute the normals
	/*cout << "Computing cloud normals..." << endl;*/
	ne.compute(*cloud_normals);
	/*cout << "Normals computed!" << endl;*/

	//// Copy the normals to a PointNormal type (so that we can get the XYZ attributes later)
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::copyPointCloud(*cloud_normals, *cloud_with_normals);
	////pcl::concatenateFields(*input, *cloud_normals, *cloud_with_normals);
	////////* cloud_with_normals = cloud + normals

	//// Get the XYZ attributes for cloud_with_normals
	//for (size_t i = 0; i<cloud_normals->points.size(); ++i)
	//{
	//	cloud_with_normals->points[i].x = cloud->points[i].x;
	//	cloud_with_normals->points[i].y = cloud->points[i].y;
	//	cloud_with_normals->points[i].z = cloud->points[i].z;
	//}

  return (cloud_normals);
}