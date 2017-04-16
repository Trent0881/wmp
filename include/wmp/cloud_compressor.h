// Cloud Compressor object header for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/common.h>

class CloudCompressor
{
public:
	//CloudCompressor();

	bool setCloud(PointCloud);
	bool compress();

private:

	PointCloud uncompressed_cloud;
	PointCloud compressed_cloud;
};