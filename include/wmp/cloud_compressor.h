// Cloud Compressor object header for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/common.h>

class CloudCompressor
{
public:

	CloudCompressor(float, float);

	bool setCloud(PointCloud);
	bool compressFlat();
	bool compressToGrid();

	PointCloud getCloud();
	Grid getGrid();

private:

	PointCloud uncompressed_cloud;
	PointCloud compressed_cloud;
	Grid obstacle_grid;

	float grid_min_x;
	float grid_min_y;
	float grid_max_x;
	float grid_max_y;
	float grid_width;
	float grid_height;
};