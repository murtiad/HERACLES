#include <iostream>
#include <cstdlib>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/shot.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/vtk_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <boost/filesystem.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/gicp.h>
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

int cordon (string filePath, string filePath2)
{
	cout << "======================================================" << endl;
	cout << "CORDON (CORrespondance finDer and detectiON)" << endl;
	cout << "Finds correspondences between two point clouds" << endl;
	cout << "ver 0.1 - 8 March 2018" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "======================================================" << endl;
	cout << endl;

	//////////////////////
	////Load first file///
	//////////////////////

	cerr << "Loading " << filePath << endl;
	
	string file_extension1  = boost::filesystem::extension(filePath);
	
	PointCloud<PointXYZRGB>::Ptr input1 (new PointCloud<PointXYZRGB>);
	
	if (file_extension1 == ".las")
	{
		*input1 = las2pcd(filePath);
	}
	else if (file_extension1 == ".pcd")
	{
		pcl::io::loadPCDFile (filePath, *input1);
	}
	else
	{
		cout << "Sorry! This file extension is not supported :(" << endl;
	}  
	
	cout << "Successfully loaded " << filePath << endl;
	cout << endl;

	///////////////////////
	////Load second file///
	///////////////////////

	cerr << "Loading " << filePath2 << endl;

	string file_extension2 = boost::filesystem::extension(filePath2);
		
	PointCloud<PointXYZRGB>::Ptr input2(new PointCloud<PointXYZRGB>);

	if (file_extension2 == ".las")
	{
		*input2 = las2pcd(filePath2);
	}
	else if (file_extension2 == ".pcd")
	{
		pcl::io::loadPCDFile(filePath2, *input2);
	}
	else
	{
		cout << "Sorry! This file extension is not supported :(" << endl;
	}

	cout << "Successfully loaded " << filePath2 << endl;
	cout << endl;
	
	////////////////////////////////////////////////////////////
	//// First things first, let's compute the cloud normals! //
	////////////////////////////////////////////////////////////

	cerr << "Computing normals for both point clouds..."<< endl;
	PointCloud<Normal>::Ptr cloud_normals1(new PointCloud<Normal>);
	PointCloud<Normal>::Ptr cloud_normals2(new PointCloud<Normal>);
	cloud_normals1 = normal(input1, 0.015);
	cloud_normals2 = normal(input2, 0.015);
	cerr << "Normals computed!" << endl;
	cerr << endl;
	
	////////////////////////////////////////
	// Next, let's compute the keypoints! //
	////////////////////////////////////////

	//// temporary code for tests ///
	//PointCloud<PointXYZ>::Ptr keypoints1(new PointCloud<PointXYZ>);
	//PointCloud<PointXYZ>::Ptr keypoints2(new PointCloud<PointXYZ>);

	//string file1, file2;

	//cout << endl;
	//cout << "Enter full .pcd file path of the first keypoints: (or you can also drag the file here)" << endl;
	//getline(cin, file1);
	//cout << "Enter full .pcd file path of the second keypoints: (or you can also drag the file here)" << endl;
	//getline(cin, file2);
	//cout << endl;
	//io::loadPCDFile(file1, *keypoints1);
	//cout << "Successfully loaded " << file1 << endl;
	//io::loadPCDFile(file2, *keypoints2);
	//cout << "Successfully loaded " << file2 << endl;
	//cout << endl;

	//// end of temporary codes ///

	cout << "======================================================" << endl;
	cout << "KEYSTON (KEYpoints Simple deTectiON)" << endl;
	cout << "Detects keypoints in a point cloud" << endl;
	cout << "ver 0.1 - 8 March 2018" << endl;
	cout << "(c) Arnadi Murtiyoso" << endl;
	cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
	cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
	cout << "https://github.com/murtiad" << endl;
	cout << "======================================================" << endl;
	cout << endl;

	cout << "Computing keypoints (3D SIFT) for both point clouds..." << endl;
	cout << endl;

	PointCloud<PointXYZ>::Ptr keypoints1(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr keypoints2(new PointCloud<PointXYZ>);

	keypoints1 = keyston(input1, cloud_normals1);
	cerr << "Detected " << keypoints1->points.size() << " 3D SIFT keypoints in the first point cloud!" << endl;
	keypoints2 = keyston(input2, cloud_normals2);
	cerr << "Detected " << keypoints2->points.size() << " 3D SIFT keypoints in the second point cloud!" << endl;
	cout << endl;

	// Export keypoints to .pcd

	io::savePCDFileASCII("cordon_o_keypoints1.pcd", *keypoints1);
	cerr << "Saved " << keypoints1->points.size() << " points to cordon_o_keypoints1.pcd." << endl;

	io::savePCDFileASCII("cordon_o_keypoints2.pcd", *keypoints2);
	cerr << "Saved " << keypoints2->points.size() << " points to cordon_o_keypoints2.pcd." << endl;
	cout << endl;

	///////////////////////////////////////////////////////
	// Next step, compute the detectors for the features //
	///////////////////////////////////////////////////////

	string feature_type;
	cout << "List of implemented feature detection algorithms :" << endl;
	cout << "fpfh	-- Fast Point Feature Histogram (Rusu et al, 2009)" << endl;
	cout << "shot	-- Signature of Histograms of OrienTations (Tombari et al, 2010)" << endl;
	cout << endl;
	cout << "Choose your destiny...er...feature type : ";
	cin >> feature_type;

	PointCloud<FPFHSignature33>::Ptr features1(new PointCloud<FPFHSignature33>());
	PointCloud<FPFHSignature33>::Ptr features2(new PointCloud<FPFHSignature33>());

	PointCloud<SHOT352>::Ptr shotFeatures1(new PointCloud<SHOT352>);
	PointCloud<SHOT352>::Ptr shotFeatures2(new PointCloud<SHOT352>);

	if (feature_type == "fpfh")
	{
		cout << "======================================================" << endl;
		cout << "FENTON (FEatures and sigNatures detectiON)" << endl;
		cout << "Computes FPFH detectors for keypoints" << endl;
		cout << "ver 0.1 - 8 March 2018" << endl;
		cout << "(c) Arnadi Murtiyoso" << endl;
		cout << "PAGE Group, ICube Laboratory UMR 7357 INSA Strasbourg" << endl;
		cout << "Contact: arnadi.murtiyoso@insa-strasbourg.fr" << endl;
		cout << "https://github.com/murtiad" << endl;
		cout << "======================================================" << endl;
		cout << endl;

		cout << "Computing feature detectors (FPFH) for both keypoint sets..." << endl;
		cout << endl;

		features1 = fenton(keypoints1, input1, cloud_normals1);
		cerr << "FPFH detectors computed for the first set of keypoints!" << endl;
		features2 = fenton(keypoints2, input2, cloud_normals2);
		cerr << "FPFH detectors computed for the second set of keypoints!" << endl;
		cerr << endl;
	}
	else if (feature_type == "shot")
	{
		// Setup the SHOT features
		//typedef pcl::FPFHSignature33 ShotFeature; // Can't use this, even despite: http://docs.pointclouds.org/trunk/structpcl_1_1_f_p_f_h_signature33.html
		//typedef SHOT352 ShotFeature;

		// Suppresses the RGB 
		PointCloud<PointXYZ>::Ptr cloud1(new PointCloud<PointXYZ>);
		copyPointCloud(*input1, *cloud1);

		PointCloud<PointXYZ>::Ptr cloud2(new PointCloud<PointXYZ>);
		copyPointCloud(*input2, *cloud2);



		SHOTEstimation<PointXYZ, Normal, SHOT352> shotEstimation1;
		search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
		shotEstimation1.setInputCloud(keypoints1);
		shotEstimation1.setSearchSurface(cloud1);
		shotEstimation1.setInputNormals(cloud_normals1);

		// Use the same KdTree from the normal estimation
		shotEstimation1.setSearchMethod(tree);
		shotEstimation1.setRadiusSearch (0.5);
		shotEstimation1.setKSearch(0);

		// Actually compute the spin images
		shotEstimation1.compute(*shotFeatures1);
		cerr << "SHOT detectors computed for the first set of keypoints!" << endl;

		SHOTEstimation<PointXYZ, Normal, SHOT352> shotEstimation2;
		shotEstimation2.setInputCloud(keypoints2);
		shotEstimation2.setSearchSurface(cloud2);
		shotEstimation2.setInputNormals(cloud_normals2);

		// Use the same KdTree from the normal estimation
		shotEstimation2.setSearchMethod(tree);
		shotEstimation2.setRadiusSearch (0.5);
		shotEstimation2.setKSearch(0);

		// Actually compute the spin images
		shotEstimation2.compute(*shotFeatures2);
		cerr << "SHOT detectors computed for the first set of keypoints!" << endl;
	}
	else
	{
		cout << "Sorry, that algorithm is not yet supported :) " << endl;
	}

	/////////////////////////////////////////////////////////
	// Finally, compute the correspondance between the two //
	/////////////////////////////////////////////////////////
	cout << "Matching keypoints..." << endl;
	cout << endl;

	double CorrDistance = 1;
	//cout << "Please enter the maximum distance allowed for the correpondence, 10 is working : ";
	//// this value need to be tested again
	//cin >> CorrDistance;

	Correspondences correspondences;

	if (feature_type == "fpfh")
	{	
		registration::CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> cest;
		cest.setInputSource(features1);
		cest.setInputTarget(features2);
		// Compute *reciprocal* correspondence
		cest.determineReciprocalCorrespondences(correspondences, CorrDistance); // input here max distance for each correspondance
		std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;
		cout << endl;
		// note: instances of the class Correspondences are vectors with index of the first file, index of the second file, and distance between them
	}
	else if (feature_type == "shot")
	{
		registration::CorrespondenceEstimation<SHOT352, SHOT352> cest;
		cest.setInputSource(shotFeatures1);
		cest.setInputTarget(shotFeatures2);
		// Compute *reciprocal* correspondence
		cest.determineReciprocalCorrespondences(correspondences, CorrDistance); // input here max distance for each correspondance
		std::cout << "Found " << correspondences.size() << " correspondences." << std::endl;
		cout << endl;
		// note: instances of the class Correspondences are vectors with index of the first file, index of the second file, and distance between them
	}

	// Create an empty text file to store the correspondance coordinates
	ofstream myfile;
	myfile.open("cordon_o_correspondencelist.txt");

	// Declare variables to store the coordinates
	int index1, index2;
	double x1, y1, z1, x2, y2, z2;

	// Write the correspondence coordinates to the text file
	for (size_t i = 0; i < correspondences.size(); ++i)
	{
		// Get the index of the correspondence in the first point cloud
		index1 = correspondences[i].index_query;

		// Get the index of the correspondence in the second point cloud
		index2 = correspondences[i].index_match;

		// Access the indexed point in the input keypoints and place them in temporary variables
		x1 = keypoints1->points[index1].x;
		y1 = keypoints1->points[index1].y;
		z1 = keypoints1->points[index1].z;
		x2 = keypoints2->points[index2].x;
		y2 = keypoints2->points[index2].y;
		z2 = keypoints2->points[index2].z;

		// Write the temporary variables to the text file
		myfile << x1 << " " << y1 << " " << z1 << " " << x2 << " " << y2 << " " << z2 << endl;
	}
	cout << correspondences.size() << endl;
	myfile.close();

	cout << "Saved correspondence list to cordon_o_correspondencelist.txt." << endl;
	cout << endl;

	///////////// Rejector for correspondences and initial alignment /////////////////

	//int inlierDist;
	//cout << "Please enter the maximum distance between points for correpondence (same unit as the input), 1 is working " << endl;
	//// this value need to be tested again
	//cin >> inlierDist;
	//pcl::CorrespondencesPtr corrFiltered(new pcl::Correspondences());
	//pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
	//rejector.setInputSource(keypointMobile);
	//rejector.setInputTarget(keypointRef);
	//rejector.setInputCorrespondences(correspondences);
	//rejector.setInlierThreshold(inlierDist);
	//rejector.setMaximumIterations(1000);
	//rejector.setRefineModel(false);
	//rejector.getCorrespondences(*corrFiltered);
	//std::cout << "Found " << corrFiltered->size() << " Filtered correspondences." << std::endl;


	// Transformation 3D
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ>::Matrix4 transformations;
	registration::TransformationEstimationSVD<PointXYZ, PointXYZ> transEst(new registration::TransformationEstimationSVD<PointXYZ, PointXYZ>);
	transEst.estimateRigidTransformation(*keypoints1, *keypoints2, correspondences, transformations);

	// Transform cloud2 with initial transformations
	PointCloud<PointXYZRGB>::Ptr mobileTransformed(new PointCloud<PointXYZRGB>);
	transformPointCloud(*input1, *mobileTransformed, transformations, true);

	pcl::io::savePCDFileASCII("Mobile_transformed.pcd", *mobileTransformed);


  return 0;
}