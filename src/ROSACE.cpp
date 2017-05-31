#include <iostream>
#include <cstdlib>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/filesystem.hpp>
#include "las2pcd.h"

using namespace std;
using namespace pcl;

/////////////////////////////////////////////////////////////////////////////////////////////////
// ROSACE (Ransac Object SegmentAtion Clearly simplE) - Performs basic RANSAC plane segmentation
// ver 0.1 - 31 May 2017 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
/////////////////////////////////////////////////////////////////////////////////////////////////
// Input : Full file path of the .pcd file
// Output : .pcd file of the inliers (rosace_o.pcd)
/////////////////////////////////////////////////////////////////////////////////////////////////

int rosace (string filePath)
{
	cout << "=====================================================" << endl;
	cout << "ROSACE (Ransac Object SegmentAtion Clearly simplE)" << endl;
	cout << "Performs basic RANSAC plane segmentation" << endl;
	cout << "ver 0.1 - 31 May 2017" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "=====================================================" << endl;
	cout << endl;

	std::cerr << "Loading " << filePath << std::endl;

	string file_extension  = boost::filesystem::extension(filePath);

	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	
	if (file_extension == ".las")
	{
		*cloud = las2pcd(filePath);
	}
	else if (file_extension == ".pcd")
	{
		pcl::io::loadPCDFile (filePath, *cloud);
	}
	else
	{
		cout << "Sorry! This file extension is not supported :(" << endl;
	}  

	cout << "Successfully loaded " << filePath << endl;
	
	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointIndices::Ptr inliers (new PointIndices);
	
	// Create the segmentation object
	SACSegmentation<PointXYZRGB> seg;
	
	// Optional
	seg.setOptimizeCoefficients (true);
	
	// Mandatory: sets the reference model and the segementation algorithm
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	
	// Set distance threshold  !!!! HERE !!!!
	seg.setDistanceThreshold (0.1);
	
	cout << "Segmenting point cloud... " << endl;
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	
	cout << "Segmentation completed!" << endl;

	if (inliers->indices.size () == 0)
	{
	  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	  return (-1);
	}
	
	int sizeinlier = inliers->indices.size ();
	
	std::cerr << "Fitted planar model coefficients: " << coefficients->values[0] << " " 
	                                    << coefficients->values[1] << " "
	                                    << coefficients->values[2] << " " 
	                                    << coefficients->values[3] << std::endl;
	
	std::cerr << "Number of inlier points: " << sizeinlier << std::endl;

	PointCloud<PointXYZRGB> cloud_o;

	cloud_o.width    = sizeinlier;				// This means that the point cloud is "unorganized"
	cloud_o.height   = 1;						// (i.e. not a depth map)
	cloud_o.is_dense = false;
	cloud_o.points.resize (cloud_o.width * cloud_o.height);

	for (size_t i = 0; i < sizeinlier; ++i)
	 {
	cloud_o.points[i].x = cloud->points[inliers->indices[i]].x;
	cloud_o.points[i].y = cloud->points[inliers->indices[i]].y;
    cloud_o.points[i].z = cloud->points[inliers->indices[i]].z;

	cloud_o.points[i].r = cloud->points[inliers->indices[i]].r;
	cloud_o.points[i].g = cloud->points[inliers->indices[i]].g;
    cloud_o.points[i].b = cloud->points[inliers->indices[i]].b;
	 }
	
	io::savePCDFileASCII ("rosace_o.pcd", cloud_o);
	cerr << "Saved " << cloud_o.points.size () << " inlier points to rosace_o.pcd." << endl;
	cerr << "TIPS: Use PARVIS to visualize the result!" << endl;
	
	return (0);
}