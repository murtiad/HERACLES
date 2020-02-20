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
// KEYSTON - Detects keypoints in a point cloud
// ver 0.1 - 8 March 2018 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : Full file path of the .pcd file
// Output : Meshed 3D model in .vtk format
////////////////////////////////////////////////////////////////////////

PointCloud<PointXYZ>::Ptr keyston (PointCloud<PointXYZRGB>::Ptr input, PointCloud<Normal>::Ptr cloud_normals)
{
	//cout << "======================================================" << endl;
	//cout << "KEYSTON (KEYpoints Simple deTectiON)" << endl;
	//cout << "Detects keypoints in a point cloud" << endl;
	//cout << "ver 0.1 - 8 March 2018" << endl;
	//cout << "(c) Arnadi Murtiyoso" << endl;
	//cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	//cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	//cout << "https://github.com/murtiad" << endl;
	//cout << "======================================================" << endl;
	//cout << endl;
	
	// Suppresses the RGB for normal computation
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
	pcl::copyPointCloud(*input,*cloud);
	
	//// Concatenate the XYZ and normal fields
	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	copyPointCloud(*cloud_normals, *cloud_with_normals);

	// Get the XYZ attributes for cloud_with_normals
	for (size_t i = 0; i<cloud_normals->points.size(); ++i)
	{
		cloud_with_normals->points[i].x = cloud->points[i].x;
		cloud_with_normals->points[i].y = cloud->points[i].y;
		cloud_with_normals->points[i].z = cloud->points[i].z;
	}

	//// DETECT SIFT KEYPOINTS

	// Estimate sift keypoints using normals values 
	SIFTKeypoint<PointNormal, PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> resultTemp;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(0.005, 8, 8);
	sift.setMinimumContrast(0.001);
	sift.setInputCloud(cloud_with_normals);
	sift.compute(resultTemp);

	// Copying the pointwithscale to point xyz so as visualize the cloud
	PointCloud<PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(resultTemp, *keypoints);
	//cerr << "Detected " << keypoints->points.size() << " 3D SIFT keypoints!" << endl;


	// Export of a ".pcd" file

	//pcl::io::savePCDFileASCII("keyston_o_keypoints.pcd", *keypoints);
	//cerr << "Saved " << keypoints->points.size() << " keypoints to keyston_o_keypoints.pcd." << endl;
	
  return (keypoints);
}