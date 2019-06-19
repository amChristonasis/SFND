/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"


void proximity(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int id, std::vector<int>& cluster, std::vector<int>& proccessed_ids)
{
  proccessed_ids.push_back(id);
  cluster.push_back(id);
  std::vector<int> nearby = tree->search(points[id],distanceTol);
  for (auto it = nearby.begin(); it != nearby.end(); ++it)
  {
    if (!((std::find(proccessed_ids.begin(), proccessed_ids.end(), *it) != proccessed_ids.end())))
    {
      proximity(points, tree, distanceTol, *it, cluster, proccessed_ids);
    }
  }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
  std::vector<int> proccessed_ids;

  for (int i = 0; i < points.size(); ++i)
  {
    if (!((std::find(proccessed_ids.begin(), proccessed_ids.end(), i) != proccessed_ids.end())))
    {
      std::vector<int> cluster;
      proximity(points, tree, distanceTol, i, cluster, proccessed_ids);
      clusters.push_back(cluster);
    }
  }
 
	return clusters;

}

int main ()
{

	// Create viewer
	

	// Create data
	std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	KdTree* tree = new KdTree;
  
    // Create KDtre
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],0));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud,"data");
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
