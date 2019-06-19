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


template<typename PointT>
std::unordered_set<int> ransac3D(pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    auto startTime = std::chrono::steady_clock::now();

    int cpt_max = 0;
    for (int it = 0; it < maxIterations; it ++){
        std::unordered_set<int> inliersResultI;
        int n1(rand() % cloud->points.size());
        int n2(rand() % cloud->points.size());
        int n3(rand() % cloud->points.size());
        float coeff_a = (cloud->points[n2].y - cloud->points[n1].y)*(cloud->points[n3].z-cloud->points[n1].z)-
           (cloud->points[n2].z-cloud->points[n1].z)*(cloud->points[n3].y-cloud->points[n1].y);
        float coeff_b = (cloud->points[n2].z - cloud->points[n1].z)*(cloud->points[n3].x-cloud->points[n1].x)-
           (cloud->points[n2].x-cloud->points[n1].x)*(cloud->points[n3].z-cloud->points[n1].z);
        float coeff_c = (cloud->points[n2].x - cloud->points[n1].x)*(cloud->points[n3].y-cloud->points[n1].y)-
           (cloud->points[n2].y-cloud->points[n1].y)*(cloud->points[n3].x-cloud->points[n1].x);
        float coeff_d = -(coeff_a*cloud->points[n1].x+coeff_b*cloud->points[n1].y+coeff_c*cloud->points[n1].z);
        int cpt = 0;
        for (int i = 0; i < cloud->points.size (); ++i){
            if (i == n1 || i == n2 | i == n3)
                continue;
            float distance_to_plan = std::fabs(coeff_a*cloud->points[i].x+coeff_b*cloud->points[i].y+coeff_c*cloud->points[i].z+coeff_d)/
               std::sqrt(coeff_a*coeff_a+coeff_b*coeff_b+coeff_c*coeff_c);
            if (distance_to_plan <= distanceTol)
            {
             ++cpt;
             inliersResultI.insert(i);
            }
        }
        if (cpt > cpt_max){
        inliersResult = inliersResultI;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac calulation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}

// template<typename PointT>
// std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
// {
//   std::unordered_set<int> inliersResult;
//   srand(time(NULL));
  
//   // TODO: Fill in this function

//   // For max iterations
//   int cpt_max = 0;
//   for (int it = 0; it < maxIterations; it ++){
//     std::unordered_set<int> inliersResultI;
//     int n1(rand() % cloud->points.size());
//     int n2(rand() % cloud->points.size());
//     float coeff_a = cloud->points[n1].y - cloud->points[n2].y;
//     float coeff_b = cloud->points[n2].x - cloud->points[n1].x;
//     float coeff_c = cloud->points[n1].x*cloud->points[n2].y - cloud->points[n2].x*cloud->points[n1].y;
//     int cpt = 0;
//     for (int i = 0; i < cloud->points.size (); ++i){
//       if (i == n1 || i == n2)
//         continue;
//       if (std::fabs(coeff_a*cloud->points[i].x+coeff_b*cloud->points[i].y+coeff_c)/std::sqrt(coeff_a*coeff_a+coeff_b*coeff_b) <= distanceTol)
//       {
//         ++cpt;
//         inliersResultI.insert(i);
//       }
//     }
//     if (cpt > cpt_max){
//       inliersResult = inliersResultI;
//     }
//   }
  
//   return inliersResult;

// }

template<typename PointT>
std::pair<pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr> segmentCloud(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol )
{

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = ransac3D(cloud, maxIterations, distanceTol);

    typename pcl::PointCloud<PointT>::Ptr plan_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            plan_cloud->points.push_back(point);
        else
            obstacles_cloud->points.push_back(point);
    }
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud, plan_cloud);
    
    return segResult;

}