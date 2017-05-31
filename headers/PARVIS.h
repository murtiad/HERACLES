#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// PARVIS (Pcds ARe VISualized) - Visualizes .pcd (PCL-compatible) point clouds
// ver 0.1 - 30 May 2017 
// (c) Arnadi Murtiyoso
// PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg
// contact: arnadi.murtiyoso@insa-strasbourg.fr
// https://github.com/murtiad
////////////////////////////////////////////////////////////////////////////////
// Input : Full file path of the .pcd file
// Output : Visualization interface (PCL's CloudViewer)
// see also http://pointclouds.org/documentation/tutorials/cloud_viewer.php
////////////////////////////////////////////////////////////////////////////////

int parvis (string filePath);