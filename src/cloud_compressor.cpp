// Cloud Compressor object definitions for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/cloud_compressor.h>

CloudCompressor::CloudCompressor(float grid_size_1, float grid_size_2)
{
	obstacle_grid.header.frame_id = "lidar_link";

	obstacle_grid.info.width = grid_size_1;
	obstacle_grid.info.height = grid_size_2;

	//obstacle_grid.info.origin.position = Point(0,0,0);
	//obstacle_grid.info.origin.orientation = Quaternion(0,0,0,1);

}

bool CloudCompressor::setCloud(PointCloud input_cloud)
{
	uncompressed_cloud = input_cloud;
	// Parameterize below?
	uncompressed_cloud.header.frame_id = "lidar_link";
	compressed_cloud.header.frame_id = "lidar_link";
	
	return true;
}

bool CloudCompressor::compressFlat()
{
	Point compressed_point;
	for(int i = 0; i < uncompressed_cloud.points.size(); i++)
	{
		compressed_point = Point(uncompressed_cloud.points[i].x, uncompressed_cloud.points[i].y, 0);
		compressed_cloud.push_back(compressed_point);
	} 

	return true;
}

bool CloudCompressor::compressToGrid()
{

	grid_min_x = 0;
	grid_min_y = 0;
	grid_max_x = 0;
	grid_max_y = 0;

	for(int i = 0; i < compressed_cloud.points.size(); i++)
	{
		if (grid_min_x > compressed_cloud.points[i].x)
		{
			grid_min_x = compressed_cloud.points[i].x;
		}
		else if (grid_max_x < compressed_cloud.points[i].x)
		{
			grid_max_x = compressed_cloud.points[i].x;
		}
		
		if (grid_min_y > compressed_cloud.points[i].y)
		{
			grid_min_y = compressed_cloud.points[i].y;
		} 
		else if (grid_max_y < compressed_cloud.points[i].y)
		{
			grid_max_y = compressed_cloud.points[i].y;
		}
	}

	ROS_INFO("Grid is %f by %f meters in size!", grid_max_x - grid_min_x, grid_max_y - grid_min_y);
	
	obstacle_grid.info.width = grid_max_x - grid_min_x;
	obstacle_grid.info.height = grid_max_y - grid_min_y;

	return true;
}

Grid CloudCompressor::getGrid()
{
	return obstacle_grid;
}

PointCloud CloudCompressor::getCloud()
{
	return compressed_cloud;
}