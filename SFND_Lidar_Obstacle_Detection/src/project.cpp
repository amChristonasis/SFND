/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
// {

//     Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
//     Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
//     Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
//     Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
//     std::vector<Car> cars;
//     cars.push_back(egoCar);
//     cars.push_back(car1);
//     cars.push_back(car2);
//     cars.push_back(car3);

//     if(renderScene)
//     {
//         renderHighway(viewer);
//         egoCar.render(viewer);
//         car1.render(viewer);
//         car2.render(viewer);
//         car3.render(viewer);
//     }

//     return cars;
// }

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block -----
    // ----------------------------------------------------
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    // renderPointCloud(viewer,inputCloud,"inputCloud");
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (0, -30, -6, 1), Eigen::Vector4f ( 10, 30, 6, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");
    
    // TODO : Change in ransac3D
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = segmentCloud(filterCloud, 100, 0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    Box box_roof;
    box_roof.x_min = -1.5;
    box_roof.x_max = 2.6;
    box_roof.y_min = -1.7;
    box_roof.y_max = 1.7;
    box_roof.z_min = -1;
    box_roof.z_max = -0.4;
    // renderBox(viewer,box_roof,1,Color(0,1,1));

    // TODO : Change with euclidian clustering kd_tree
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "cluster size ";
    //     pointProcessorI->numPoints(cluster);
    //     Box box = pointProcessorI->BoundingBox(cluster);
    //     renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
    //     renderBox(viewer,box,clusterId);
    //     ++clusterId;
    // }
  
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
    
//     // RENDER OPTIONS
//     bool renderScene = false;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
    
//     Lidar* lidar = new Lidar(cars,0);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud = lidar->scan();
//     // renderRays(viewer, lidar->position, scan_cloud);
//     // renderPointCloud(viewer, scan_cloud, "scan_cloud");

//     ProcessPointClouds<pcl::PointXYZ>* processPointClouds = new ProcessPointClouds<pcl::PointXYZ>();

//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds->SegmentPlane(scan_cloud, 100, 0.2);
//     // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//     renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds->Clustering(segmentCloud.first, 2.0, 3, 30);

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//         std::cout << "cluster size ";
//         processPointClouds->numPoints(cluster);
//         BoxQ box = processPointClouds->BoundingBoxQ(cluster);
//         renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//         renderBox(viewer,box,clusterId);
//         ++clusterId;
//     }
  
// }


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}