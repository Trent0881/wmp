// Cloud Compressor object definitions for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated April 26 2017 by Trent Ziemer

#include <wmp/cloud_compressor.h>

CloudCompressor::CloudCompressor(float grid_size_1, float grid_size_2, float x, float y)
{
	obstacle_grid.header.frame_id = "lidar_link";

	obstacle_grid.info.width = grid_size_1;
	obstacle_grid.info.height = grid_size_2;

	x_offset = x;
	y_offset = y;
}

bool CloudCompressor::setCloud(PointCloud input_cloud)
{
	uncompressed_cloud = input_cloud;
	// Ideally parameterize things like the below frame IDs
	uncompressed_cloud.header.frame_id = "lidar_link";
	compressed_cloud.header.frame_id = "lidar_link";

	// Set pose of the map wrt the world, to correct for offsetting, and a weird x-y axis rotation that is innate
	geometry_msgs::Pose examplePose;

	geometry_msgs::Quaternion quat;
    quat.x = 1;
    quat.y = 1;
    quat.z = 0;
    quat.w = 0;
    examplePose.orientation = quat;

    geometry_msgs::Point pt;
    pt.x = x_offset; 
    pt.y = y_offset; 
    examplePose.position = pt;

	obstacle_grid.info.origin = examplePose;
	
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

 	// NEW!
	grid_min_x = -2.5;
	grid_max_x = 2.5;
	grid_min_y = -2.5;
	grid_max_y = 2.5;

	ROS_INFO("Grid is %f by %f meters in size!", grid_max_x - grid_min_x, grid_max_y - grid_min_y);

	ROS_INFO("and number of cells is %i by %i!", obstacle_grid.info.width, obstacle_grid.info.height);

	grid_width = grid_max_x - grid_min_x;
	grid_height = grid_max_y - grid_min_y;

	obstacle_grid.info.resolution = grid_width/obstacle_grid.info.width;

	ROS_INFO("Resolution (probs width based) is %f", obstacle_grid.info.resolution);

	const float cell_size_x = grid_width/obstacle_grid.info.width;
	const float cell_size_y = cell_size_x; // This is a forcing thing, so it is square not rectangular. Not happy about it.

	ROS_INFO("Cell size is thus %f by %f meters in size!", cell_size_x, cell_size_y);

	float cell_min_x;
	float cell_max_x;
	float cell_min_y;
	float cell_max_y;

	int points_in_cell;

	ROS_INFO("Cloud size: %lu", compressed_cloud.points.size());
	
	float x_max = (float) obstacle_grid.info.width;
	float x_min = - (float) obstacle_grid.info.width;
	float y_max = (float) obstacle_grid.info.height;
	float y_min = - (float) obstacle_grid.info.height;

	cell_min_x = grid_min_x;
	cell_max_x = grid_min_x;
	obstacle_grid.data.clear();
	
	while(cell_max_x < grid_max_x)
	{

		cell_min_x = cell_max_x;
		cell_max_x = cell_min_x + cell_size_x;

		cell_min_y = grid_min_y;
		cell_max_y = grid_min_y;

		while(cell_max_y < grid_max_y)
		{		
			cell_min_y = cell_max_y;
			cell_max_y = cell_min_y + cell_size_y;

			for(int k = 0; k < compressed_cloud.points.size(); k++)
			{

				if( compressed_cloud.points[k].x >= cell_min_x
				 && compressed_cloud.points[k].x <= cell_max_x
				 && compressed_cloud.points[k].y >= cell_min_y
				 && compressed_cloud.points[k].y <= cell_max_y)
				{
					points_in_cell++;
					// Either or works, mate 
					//points_in_cell = 100;
				}
			}

			obstacle_grid.data.push_back(30*sqrt(sqrt(points_in_cell))); // I HOPE THIS DOESNT EXCEED 100

			points_in_cell = 0;
		}
	}
	obstacle_grid.data[0] = 0; // yes
	ROS_INFO("Grid size: %lu", obstacle_grid.data.size());

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