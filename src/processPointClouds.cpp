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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create Voxel Grid
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    //std::cout << typeid(vg).name() << endl;
    // Set the input to the Voxel Grid
    vg.setInputCloud(cloud);
    // Define the cell size
    vg.setLeafSize(filterRes,filterRes,filterRes);
    // Save the results to cloudFiltered
    vg.filter(*cloudFiltered);
    // std::cerr << "PointCloud after voxel grid point reduction: " << cloudFiltered->width * cloudFiltered->height 
    //    << " data points (" << pcl::getFieldsList (*cloudFiltered) << ")." << std::endl;

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    // Set region to true for dealing with points inside the CropBox
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    // Set the input cloud to the cloudFiltered from the Voxel Grid
    region.setInputCloud(cloudFiltered);
    // Save the results in the cloud region
    // Crop and save the points left inside the box region
    region.filter(*cloudRegion);
    // std::cerr << "PointCloud after region of interest: " << cloudRegion->width * cloudRegion->height 
    //    << " data points (" << pcl::getFieldsList (*cloudRegion) << ")." << std::endl;

    // Optional to remove the roof points
    // Use CropBox to remove the points outside the region
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    // Results are indices of the points inside the cloud region
    // that fit inside the box
    std::vector<int> indices;
    roof.filter(indices);

    // Add the indice of points inside the cloud region
    // to the inliers vector
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point:indices)
        inliers->indices.push_back(point);
    
    // Separate point cloud using segmentation
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    // Set indices of roofPoints
    extract.setIndices(inliers);
    // Set negative to remove these roof points
    extract.setNegative(true);
    // Extract indices from updated inliers without the roof points
    extract.filter(*cloudRegion);
    // std::cerr << "PointCloud after roof points removed: " << cloudRegion->width * cloudRegion->height 
    //    << " data points (" << pcl::getFieldsList (*cloudRegion) << ").";

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new typename pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new typename pcl::PointCloud<PointT> ());

    // Add plane clouds
    for(int index: inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    // see PCL tutorial for hints: http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Create the inliers for separating the pointClouds
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create coefficients for rendering the plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

    // Optional parameters
    seg.setOptimizeCoefficients (true);
    // Mandatory parameters
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

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
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Feed in the cloud to the function for the KD tree
    tree->setInputCloud(cloud);

    // Create cluster Indices
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    // Feed in the cloud to the cluster
    ec.setInputCloud(cloud);
    // Generate the cluster indices using the `extract` method
    ec.extract(clusterIndices);

    // Iterating cluster indices and create some point clouds
    for (pcl::PointIndices getIndices:clusterIndices)
    {
        // Create new cloud cluster
        typename pcl::PointCloud<PointT>:: Ptr cloudCluster (new pcl::PointCloud<PointT>);

        // Iterating each cluster index of the obstacle cloud
        // get its members of points and add to the cloud cluster
        for (int index:getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
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
    // std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// RANSAC Plane process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	while(maxIterations--)
	{
		// Randomly sample subset (pick three points)
		// Randomly pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3 )
			inliers.insert(rand()%(cloud->points.size()));
		
		// 3D points
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		// First point (x1,y1,z1)
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr++;
		// Second point (x2,y2,z2)
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		// Third point (x3,y3,z3)
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Normal vector
		float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

		// Plane fit polynomial
		float A = i;
		float B = j;
		float C = k;
		float D = -(i*x1 + j*y1 + k*z1);

		// Iterate through all points in the pointCloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// Continue and skip to the next pointCloud if
			// the point is already part of the line
			if(inliers.count(index) > 0)
			{
				continue;
			}

            PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			// Measure distance between every point and fitted line
			float d = fabs(A*x4 + B*y4 + C*z4 + D) / sqrt(A*A + B*B + C*C);

			// If distance is smaller than threshold count it as inlier
			if (d <= distanceTol)
				inliers.insert(index);
		}

		// Update the inliers results when the new inliers
		// is greater than the best inliers count so far
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	// Stop the timer
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "RANSAC Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	// Mark the point as being processed
	processed[indice] = true;
	// Add the point back to the cluster
	cluster.push_back(indice);
	
	// Find the list of indices of the nearby points
	// using tree search with indice and distance tolerance
	std::vector<int> nearest = tree->search(points[indice],distanceTol);
    
	// Iterate through nearby indices
	for (int id:nearest)
	{
		// If the point has not been processed yet,
		// then pass in the point ID and other parameters.
		// The function runs recursively to build up the cluster
		if(!processed[id])
			ProcessPointClouds<PointT>::proximity(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
 
	// Create a vector of boolean to keep track of which points have been processed. 
	// The size of boolean vector will match the same size as points.
	// Initialize processed vector to be false as the points are not yet processed
	std::vector<bool> processed(points.size(),false);
	
	int i = 0;
	while (i < points.size())
	{
		// If the point has been processed already
		// move on to next point, increment the point and continue
		if(processed[i])
		{
			i++;
			continue;
		}
		
		// Otherwise if the point has not yet been processed
		// create a new cluster of ints
		std::vector<int> cluster;
		
		// Calling proximity function
		ProcessPointClouds<PointT>::proximity(i, points, cluster, processed, tree, distanceTol);

        if (cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            // Add cluster to the vector of clusters
            clusters.push_back(cluster);
        }
		i++;
	}

	return clusters;
}