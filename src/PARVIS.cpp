#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include "las2pcd.h"

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

int user_data;
    
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.5, 0.8, 0.9);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
	//std::cout << "PCL INFO : I only run once ;)" << std::endl;
    
}
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}
    
int parvis (string filePath)
{
	cout << "=========================================================================" << endl;
	cout << "PARVIS (Pcds ARe VISualized)" << endl;
	cout << "Visualizes .pcd (PCL-compatible) point clouds" << endl;
	cout << "ver 0.1 - 29 May 2017" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "see also http://pointclouds.org/documentation/tutorials/cloud_viewer.php" << endl;
	cout << "=========================================================================" << endl;
	cout << endl;
	std::cerr << "Loading " << filePath << std::endl;
	cout << "Please wait...";

	string file_extension  = boost::filesystem::extension(filePath);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

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
    
    pcl::visualization::CloudViewer viewer("PARVIS v0.1");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
	cout << endl;
	cout << "Successfully loaded " << filePath << endl;

    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    }
    return 0;
}