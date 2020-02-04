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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Load Point Cloud Data (PCD) - now disable as the PCD will be stream from main()
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // DEBUG: render the input PCD
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    /*************************************************************************
    | Step 0. Preprocess PCD: downsampling and select region of interest     |
    /************************************************************************/

    // Voxel grid size [meters]
    float VoxelGridSize = 0.4;
    
    // Region of interest min and max points
    // [offset from the center of the car in meters]
    float min_x = -10;
    float min_y = -5.5;
    float min_z = -2;
    float max_x = 30;
    float max_y = 7.5;
    float max_z = 0.5;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud,VoxelGridSize,
                                                                                    Eigen::Vector4f(min_x,min_y,min_z,1),
                                                                                    Eigen::Vector4f(max_x,max_y,max_z,1));
    // DEBUG: render the down-sampled cloud
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    /*************************************************************************
    | Step 1. Segment the filtered cloud into two parts, road and obstacles. |
    /************************************************************************/

    /* Segment PCD using RANSAC (3D) plane algorithm */
	int maxIterations = 100;
    float RANSAC_distanceTol = 0.3;
	std::unordered_set<int> inliers = pointProcessorI->RansacPlane(filterCloud,maxIterations,RANSAC_distanceTol);
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

	// Render road plane (inliers) in green 
    // and obstacle (outliers) in red
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"Plane Cloud",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"Obstacle Cloud",Color(1,0,0));
	}
  	else
  	{
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
  		renderPointCloud(viewer,filterCloud,"data");
  	}

    /*
    // Bounding box of the ego car roof
    Box ego_carRoof;
    ego_carRoof.x_min = -1.5;
    ego_carRoof.y_min = -1.7;
    ego_carRoof.z_min = -1;
    ego_carRoof.x_max = 2.6;
    ego_carRoof.y_max = 1.7;
    ego_carRoof.z_max = -0.4;
    Color purple = Color(128,0,128);
    renderBox(viewer,ego_carRoof,999,purple);
    */

    /********************************************
    | Step 2. Cluster the obstacle cloud        |
    /*******************************************/

    auto startTime = std::chrono::steady_clock::now();

    // Inserting obstacle cloud into KD-Tree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    int i = 0;
    for (auto point : cloudOutliers->points) {
        const std::vector<float> p{ point.x, point.y, point.z };
        tree->insert(p, i++);
        points.push_back(p);
    }

  	float cluster_distanceTol = 0.5;
    int minSize = 8;
    int maxSize = 999;
  	std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(points, tree, cluster_distanceTol, minSize, maxSize);
  	
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "Found " << clusters.size() << " clusters and took " << elapsedTime.count() << " milliseconds" << std::endl;

    /************************************************
    | Step 3. Find bounding boxes for the clusters  |
    /***********************************************/

	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
	ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    int clusterId = 0;
  	for(std::vector<int> cluster:clusters)
  	{
        // Create clusters cloud
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		
        // Render clusters in cycled colors, red, yellow, and blue
		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);

		// Render bounding box around the clusters in red
        Box box = pointProcessor->BoundingBox(clusterCloud);
        renderBox(viewer,box,clusterId,Color(1,0,0));

        // Bounding box color cycle red, green blue
        // renderBox(viewer,box,clusterId,colors[clusterId%colors.size()]);

  		++clusterId;
  	}

    // Render the obstacle cloud without bounding boxes if no clusters are found
  	if(clusters.size() == 0)
  		renderPointCloud(viewer,cloudOutliers,"data");
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

    // Simulated highway point cloud
    // simpleHighway(viewer);

    // Create file stream for process point cloud
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    
    // PCD dataset 1: Cars in city blocks
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // PCD dataset 2: Cars and cyclist in city blocks
    // std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // Real world lidar city block
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}
