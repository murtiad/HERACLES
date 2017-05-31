#include <iostream>
#include <cstdlib>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

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

int rosace (string filePath);