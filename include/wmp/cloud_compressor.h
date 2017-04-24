// Cloud Compressor object header for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated April 24 2017 by Trent Ziemer

#include <wmp/common.h>

class CloudCompressor
{
public:

	CloudCompressor(float, float, float, float);

	PointCloud getCloud();
	Grid getGrid();

	bool setCloud(PointCloud);
	bool compressFlat();
	bool compressToGrid();

	PointCloud new_cloud;

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

	float x_offset;
	float y_offset;
};
