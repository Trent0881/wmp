// Cloud Compressor object definitions for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/cloud_compressor.h>

CloudCompressor::CloudCompressor(float total_grid_cells)
{
	
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
	return true;
}

PointCloud CloudCompressor::getCloud()
{
	return compressed_cloud;
}