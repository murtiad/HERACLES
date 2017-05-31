#pragma once
#include <iostream>
#include <cstdlib>
#include <liblas/liblas.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

////////////////////////////////////////////////////////////////////////
// LAS2PCD - Converts .las point clouds into PCL-friendly format .pcd
// ver 0.2 - 29 May 2017 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : Full file path of the .las file
// Output : - Object from class pcl::PointCloud<pcl::PointXYZRGB>
//			- .pcd file 
////////////////////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZRGB> las2pcd (string filePath);