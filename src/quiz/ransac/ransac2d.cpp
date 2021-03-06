/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
	// RANSAC process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--)
	{
		// Randomly sample subset (pick two points)
		// Randomly pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() <2)
			inliers.insert(rand()%(cloud->points.size()));
		
		// 2D points
		float x1, y1, x2, y2;

		// First point (x1,y1)
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		// Second point (x2,y2)
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// Line fit polynomial
		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		// Iterate through all points in the pointCloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// Continue and skip to the next pointCloud if
			// the point is already part of the line
			if(inliers.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			// Measure distance between every point and fitted line
			float d = fabs(a*x3+b*y3+c) / sqrt(a*a+b*b);

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
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	// RANSAC Plane process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
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

			pcl::PointXYZ point = cloud->points[index];
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
    std::cout << "RANSAC Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	/* FOR RANSAC 2D ONLY */
	// //Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// // TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	/* FOR RANSAC 3D ONLY */
	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
