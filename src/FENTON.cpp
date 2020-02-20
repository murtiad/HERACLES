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
// FENTON - Computes FPFH detectors for keypoints
// ver 0.1 - 8 March 2018 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : 
// Output : 
////////////////////////////////////////////////////////////////////////

PointCloud<FPFHSignature33>::Ptr fenton (PointCloud<PointXYZ>::Ptr keypoints, PointCloud<PointXYZRGB>::Ptr input, PointCloud<Normal>::Ptr cloud_normals)
{
	//cout << "======================================================" << endl;
	//cout << "FENTON (FEatures and sigNatures detectiON)" << endl;
	//cout << "Computes FPFH detectors for keypoints" << endl;
	//cout << "ver 0.1 - 8 March 2018" << endl;
	//cout << "(c) Arnadi Murtiyoso" << endl;
	//cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	//cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	//cout << "https://github.com/murtiad" << endl;
	//cout << "======================================================" << endl;
	//cout << endl;

	// Suppresses the RGB 
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	pcl::copyPointCloud(*input, *cloud);

	/// DETECT FPFH FEATURES

	 // Create the FPFH estimation class, and pass the input dataset+normals to it
	 FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh;
	 fpfh.setInputCloud(keypoints);
	 fpfh.setSearchSurface(cloud);
	 fpfh.setInputNormals (cloud_normals);
	 // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

	 // Create an empty kdtree representation, and pass it to the FPFH estimation object.
	 // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	 search::KdTree<PointXYZ>::Ptr tree2 (new search::KdTree<PointXYZ>);

	 fpfh.setSearchMethod (tree2);

	 // Output datasets
	 PointCloud<FPFHSignature33>::Ptr fpfhs (new PointCloud<FPFHSignature33> ());

	 // Use all neighbors in a sphere of radius
	 // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	 fpfh.setRadiusSearch (0.5);

	 // Compute the features
	 fpfh.compute (*fpfhs);
	 /*cerr << "FPFH detectors computed for the keypoints!" << endl;*/

	// // Write the results to a .pcd file
	//io::savePCDFileASCII ("keyston_o_detectors.pcd", *fpfhs);
	//cerr << "Saved " << fpfhs->points.size() << " keypoints to keyston_o_detectors.pcd." << endl;
	//cerr << "Check: input cloud point number is " << keypoints->points.size() << endl;
	//cerr << "TIPS: Use PARVIS to visualize the result!" << endl;

	 // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
	
  return (fpfhs);
}