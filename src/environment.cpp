/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // Voxel grid size [meters]
    float VoxelGridSize = 0.3;
    
    // Region of interest min and max points
    // [offset from the center of the car in meters]
    int min_x = -10;
    int min_y = -6.7;
    int min_z = -2;
    int max_x = 30;
    int max_y = 8.5;
    int max_z = 0.5;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,VoxelGridSize,
                                                                                    Eigen::Vector4f(min_x,min_y,min_z,1),
                                                                                    Eigen::Vector4f(max_x,max_y,max_z,1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    /* Segment PCD using RANSAC (3D) plane algorithm */
	int maxIterations = 100;
    float distanceTol = 0.3;
	std::unordered_set<int> inliers = pointProcessorI->RansacPlane(filterCloud,maxIterations,distanceTol);

	pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < filterCloud->points.size(); index++)
	{
		pcl::PointXYZI point = filterCloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render cloud with inliers and outliers
	if(inliers.size())
	{
        // Color road plane in green
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        // Color obstacles in red
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,filterCloud,"data");
  	}

    // Bounding roof box of the ego car
    Box roof_box;
    roof_box.x_min = -1.5;
    roof_box.y_min = -1.7;
    roof_box.z_min = -1;
    roof_box.x_max = 2.6;
    roof_box.y_max = 1.7;
    roof_box.z_max = -0.4;
    Color purple = Color(128,0,128);
    renderBox(viewer,roof_box,999,purple);
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    
    // Rendering using renderRays
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer,lidar->position,inputCloud);
    
    // Rendering using renderPointCloud
    // pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor; // Instantiate object on stack (not recommended)
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); // Instantiate object on heap
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud,1,0.2);
    
    // Render obstacle and plane point cloud
    renderPointCloud(viewer, segmentCloud.first, "obstCloud",Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud",Color(0,1,0));

    // Feeding in the obstacle cloud with min 3 points and max 30 points consider as cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    // Vector of color in red, yellow, blue
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // Iterating through cluster
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

        // Rendering bounding box around the cluster
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId,colors[clusterId%colors.size()]);

        ++clusterId;
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

    // Simulate highway point cloud
    // simpleHighway(viewer);

    // Real world lidar city block
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}