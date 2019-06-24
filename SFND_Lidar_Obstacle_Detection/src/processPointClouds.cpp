// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int id, std::vector<int>& cluster, std::vector<int>& proccessed_ids)
{
    proccessed_ids.push_back(id);
    cluster.push_back(id);
    std::vector<float> point_tree;
    point_tree.push_back(cloud->points[id].x);
    point_tree.push_back(cloud->points[id].y);
    point_tree.push_back(cloud->points[id].z);
    std::vector<int> nearby = tree->search(point_tree,distanceTol);
    for (auto it = nearby.begin(); it != nearby.end(); ++it)
    {
        if (!((std::find(proccessed_ids.begin(), proccessed_ids.end(), *it) != proccessed_ids.end())))
        {
        proximity(cloud, tree, distanceTol, *it, cluster, proccessed_ids);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<int> proccessed_ids;

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (!((std::find(proccessed_ids.begin(), proccessed_ids.end(), i) != proccessed_ids.end())))
        {
        std::vector<int> cluster;
        proximity(cloud, tree, distanceTol, i, cluster, proccessed_ids);
        clusters.push_back(cluster);
        }
    }
    
    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Create KDtree
    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++){
        std::vector<float> point_tree;
        point_tree.push_back(cloud->points[i].x);
        point_tree.push_back(cloud->points[i].y);
        point_tree.push_back(cloud->points[i].z);
        tree->insert(point_tree,i);
    }

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //
    std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, clusterTolerance);
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Render clusters
    int clusterId = 0;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cluster_clouds;
    for(std::vector<int> cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster)
            clusterCloud->points.push_back(cloud->points[indice]);
        if (clusterCloud->points.size() < maxSize && clusterCloud->points.size() > minSize)
            cluster_clouds.push_back(clusterCloud);
        ++clusterId;
    }

    return cluster_clouds;
    
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol, int minSizePlan)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    auto startTime = std::chrono::steady_clock::now();

    int cpt_max = 0;
    while (inliersResult.size() < minSizePlan) {
        for (int it = 0; it < maxIterations; it ++){
            std::unordered_set<int> inliersResultI;
            int n1(rand() % cloud->points.size());
            int n2(rand() % cloud->points.size());
            while (n1 == n2)
                n2 = rand() % cloud->points.size();
            int n3(rand() % cloud->points.size());
            while (n3 == n2 || n3 == n1)
                n3 = rand() % cloud->points.size();
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
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac calulation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::segmentCloud(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol, int minSizePlan)
{

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = ransac3D(cloud, maxIterations, distanceTol, minSizePlan);

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


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_crop (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_crop_roof (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> crop(true);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(cloud_filtered);
    crop.filter(*cloud_crop);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,0,1));
    roof.setInputCloud(cloud_crop);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_crop);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_crop_roof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop_roof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr plan_cloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
        plan_cloud->points.push_back(cloud->points[index]);
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacles_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles_cloud, plan_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients };
    // Create the segmentation object
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
        j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointT>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointT>);
    pcl::PCA<pcl::PointT> pca;
    pca.setInputCloud(cluster);
    pca.project(*cluster, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */

   // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    // auto minPoint, maxPoint;
    auto minPoint = cloudPointsProjected->points[0];
    auto maxPoint = cloudPointsProjected->points[0];
    
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    // This viewer has 4 windows, but is only showing images in one of them as written here.
    // pcl::visualization::PCLVisualizer *visu;
    // visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
    // int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    // visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    // visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    // visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
    // visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
    // visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
    // visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);

    BoxQ boxq;
    boxq.bboxTransform = bboxTransform;
	boxq.bboxQuaternion = bboxQuaternion;
	boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}