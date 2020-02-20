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
#include "normal.h"
#include "keyston.h"
#include "fenton.h"

using namespace std;
using namespace pcl;

////////////////////////////////////////////////////////////////////////
// CORDON - Finds correspondences between two point clouds
// ver 0.1 - 8 March 2018 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////
// Input : Two overlapping point cloudsd
// Output : 
////////////////////////////////////////////////////////////////////////

int cordon (string filePath, string filePath2);