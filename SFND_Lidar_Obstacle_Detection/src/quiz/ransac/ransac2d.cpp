/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cstdlib>
#include <cmath>
#include <chrono>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
  int cpt_max = 0;
  for (int it = 0; it < maxIterations; it ++){
    std::unordered_set<int> inliersResultI;
    int n1(rand() % cloud->points.size());
    int n2(rand() % cloud->points.size());
    float coeff_a = cloud->points[n1].y - cloud->points[n2].y;
    float coeff_b = cloud->points[n2].x - cloud->points[n1].x;
    float coeff_c = cloud->points[n1].x*cloud->points[n2].y - cloud->points[n2].x*cloud->points[n1].y;
    int cpt = 0;
    for (int i = 0; i < cloud->points.size (); ++i){
      if (i == n1 || i == n2)
        continue;
      if (std::fabs(coeff_a*cloud->points[i].x+coeff_b*cloud->points[i].y+coeff_c)/std::sqrt(coeff_a*coeff_a+coeff_b*coeff_b) <= distanceTol)
      {
        ++cpt;
        inliersResultI.insert(i);
      }
    }
    if (cpt > cpt_max){
      inliersResult = inliersResultI;
    }
  }
}


// std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
// 	std::unordered_set<int> inliersResult;
// 	srand(time(NULL));
	
// 	auto startTime = std::chrono::steady_clock::now();

// 	int cpt_max = 0;
// 	for (int it = 0; it < maxIterations; it ++){
// 		std::unordered_set<int> inliersResultI;
// 		int n1(rand() % cloud->points.size());
// 		int n2(rand() % cloud->points.size());
// 		int n3(rand() % cloud->points.size());
// 		float coeff_a = (cloud->points[n2].y - cloud->points[n1].y)*(cloud->points[n3].z-cloud->points[n1].z)-
// 				(cloud->points[n2].z-cloud->points[n1].z)*(cloud->points[n3].y-cloud->points[n1].y);
// 		float coeff_b = (cloud->points[n2].z - cloud->points[n1].z)*(cloud->points[n3].x-cloud->points[n1].x)-
// 				(cloud->points[n2].x-cloud->points[n1].x)*(cloud->points[n3].z-cloud->points[n1].z);
// 		float coeff_c = (cloud->points[n2].x - cloud->points[n1].x)*(cloud->points[n3].y-cloud->points[n1].y)-
// 				(cloud->points[n2].y-cloud->points[n1].y)*(cloud->points[n3].x-cloud->points[n1].x);
// 		float coeff_d = -(coeff_a*cloud->points[n1].x+coeff_b*cloud->points[n1].y+coeff_c*cloud->points[n1].z);
// 		int cpt = 0;
// 		for (int i = 0; i < cloud->points.size (); ++i){
// 		if (i == n1 || i == n2 | i == n3)
// 			continue;
// 		float distance_to_plan = std::fabs(coeff_a*cloud->points[i].x+coeff_b*cloud->points[i].y+coeff_c*cloud->points[i].z+coeff_d)/
// 				std::sqrt(coeff_a*coeff_a+coeff_b*coeff_b+coeff_c*coeff_c);
// 		if (distance_to_plan <= distanceTol)
// 		{
// 			++cpt;
// 			inliersResultI.insert(i);
// 		}
// 		}
// 		if (cpt > cpt_max){
// 		inliersResult = inliersResultI;
// 		}
// 	}

// 	auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "Ransac calulation took " << elapsedTime.count() << " milliseconds" << std::endl;

// 	return inliersResult;
// }

int main ()
{
	std::cout << "T-1";

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	std::cout << "T0";


	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	
	std::cout << "T1";
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 1.0);
	std::cout << "T2";


	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	std::cout << "T3";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	std::cout << "T4";


	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::cout << "T5";

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
	std::cout << "T6";

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
