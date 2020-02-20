#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
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

PointCloud<PointXYZ>::Ptr keyston (PointCloud<PointXYZRGB>::Ptr input, PointCloud<Normal>::Ptr cloud_normals);