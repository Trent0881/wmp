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

	ROS_INFO("and number of cells is %i by %i!", obstacle_grid.info.width, obstacle_grid.info.height);

	grid_width = grid_max_x - grid_min_x;
	grid_height = grid_max_y - grid_min_y;

	obstacle_grid.info.resolution = grid_width/obstacle_grid.info.width;

	ROS_INFO("Cell size is thus %f by %f meters in size!", grid_width/obstacle_grid.info.width, grid_width/obstacle_grid.info.width);
	float cell_size_x = grid_width/obstacle_grid.info.width;
	float cell_size_y = cell_size_x;

	float cell_min_x = 0;
	float cell_max_x = 0;
	float cell_min_y = 0;
	float cell_max_y = 0;

	int points_in_cell;

	ROS_INFO("Good1 size: %lu", compressed_cloud.points.size());
	for (int i = 0; i < obstacle_grid.info.width; i++)
	{
		cell_min_x = cell_size_x*i;
		cell_max_x = cell_min_x + cell_size_x;
		for (int j = 0; j < obstacle_grid.info.height; j++)
		{
			cell_min_y = cell_size_y*j;
			cell_max_y = cell_min_y + cell_size_y;
			points_in_cell = 0;
			for(int k = 0; k < compressed_cloud.points.size()/2; k++)
			{
				if( compressed_cloud.points[k].x > cell_min_x
				 && compressed_cloud.points[k].x < cell_max_x
				 && compressed_cloud.points[k].y > cell_min_y
				 && compressed_cloud.points[k].y < cell_max_y)
				{
					points_in_cell++;
				}
			}
			obstacle_grid.data.push_back(points_in_cell); // I HOPE THIS DOESNT EXCEED 100
		}
	}

	ROS_INFO("Good1 size: %lu", obstacle_grid.data.size());
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