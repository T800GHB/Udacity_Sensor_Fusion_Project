/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer) {

    Car egoCar(Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::shared_ptr<Lidar> lidar (new Lidar(cars, 0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud= lidar->scan();
//    renderRays(viewer, lidar->position, input_cloud);

//    renderPointCloud(viewer, input_cloud, "input_cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_processor;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud =
            point_processor.SegmentPlane(input_cloud, 100, 0.2);
    // Draw point cloud on the ground and on the air with line blow
    if (render_obst) {
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }
    if (render_plane) {
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor.Clustering(segmentCloud.first, 1.0, 3, 100);
    int clusterId = 0;
    std::vector<Color> colors {Color(0,0,1), Color(1,1,0), Color(1,0,0), Color(0, 1, 1),  Color(1, 0, 1),
                               Color(0.5, 0, 1), Color(0.5, 1, 0.5)};

    for (const auto& cluster : cloudClusters) {
        std::cout << "Cluster size ";
        point_processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        if (render_box) {
            Box box = point_processor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }


}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
        const std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>>& pointProcessorI,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool render_obst = true;
    bool render_plane = true;
    bool render_box = true;

//    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>());
//    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud
        = pointProcessorI->FilterCloud(inputCloud, 0.1, Eigen::Vector4f (-5, -5, -2, 1),
                Eigen::Vector4f ( 20, 5, 0.5, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
            pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
//    renderPointCloud(viewer,filterCloud,"inputCloud");

    if (render_obst) {
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    }
    if (render_plane) {
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }

    Box ego{-1.5, -1.7, -1, 2.6, 1.7, -0.4};
    renderBox(viewer, ego, -1, Color(1, 0, 1));

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters
        = pointProcessorI->Clustering(segmentCloud.first, 0.5, 30, 2500);
    int clusterId = 0;
    std::vector<Color> colors {Color(0,0,1), Color(1,1,0), Color(1,0,0),
                               Color(0, 1, 1),  Color(1, 0, 1),
                               Color(0.5, 0, 1), Color(0.5, 1, 0.5)};
    for (const auto& cluster : cloudClusters) {
        std::cout << "Cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        if (render_box) {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        clusterId++;
    }
}

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
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS) {
        viewer->addCoordinateSystem(1.0);
    }
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);

//    simpleHighway(viewer);
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

//    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce (100);
    } 
}