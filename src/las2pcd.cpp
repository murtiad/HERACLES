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


pcl::PointCloud<pcl::PointXYZRGB> las2pcd (string filePath)
{
    //std::cerr << "Loading  " << filePath << std::endl;
    
    // instancing a new PCL pointcloud object
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Opening  the las file
    std::ifstream ifs(filePath.c_str(), std::ios::in | std::ios::binary);

	// Safeguard against opening failure
    if(ifs.fail()) 
	{
        std::cerr << "ERROR : Impossible to open the file : " << filePath <<std::endl;
        /*return 0;*/
    }

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs); // reading las file
    unsigned long int nbPoints=reader.GetHeader().GetPointRecordsCount();

	// Fill in the cloud data
	cloud.width    = nbPoints;				// This means that the point cloud is "unorganized"
	cloud.height   = 1;						// (i.e. not a depth map)
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);

	//cout << cloud.points.size () << " points detected in " << filePath << endl;
	//cout << "Converting...Please be patient..." ;

	int i=0;				// counter
	uint16_t r1, g1, b1;	// RGB variables for .las (16-bit coded)
	int r2, g2, b2;			// RGB variables for converted values (see below)
	uint32_t rgb;			// "packed" RGB value for .pcd

	while(reader.ReadNextPoint()) 
	{
		// get XYZ information
		cloud.points[i].x = (reader.GetPoint().GetX());
	    cloud.points[i].y = (reader.GetPoint().GetY());
	    cloud.points[i].z = (reader.GetPoint().GetZ());
				
		// get RGB information. Note: in liblas, the "Color" class can be accessed from within the "Point" class, thus the triple gets
		r1 = (reader.GetPoint().GetColor().GetRed());
		g1 = (reader.GetPoint().GetColor().GetGreen());
		b1 = (reader.GetPoint().GetColor().GetBlue()); 

		// .las stores RGB color in 16-bit (0-65535) while .pcd demands an 8-bit value (0-255). Let's convert them!
		// Note : slight rounding using ceil function...
		r2 = ceil(((float)r1/65536)*(float)256);
		g2 = ceil(((float)g1/65536)*(float)256);
		b2 = ceil(((float)b1/65536)*(float)256);

		// PCL particularity: must "pack" the RGB into one single integer and then reinterpret them as float
		rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);

		cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
					
		i++; // ...moving on
	}
	
	//pcl::io::savePCDFileASCII ("pointcloud.pcd", cloud);

	//cout << endl;
	//cerr << "Saved " << cloud.points.size () << " data points to pointcloud.pcd." << endl;

	return (cloud);
}