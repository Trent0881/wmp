// An actually Good Grid (GoodGrid) object header for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated April 26 2017 by Trent Ziemer

#include <wmp/common.h>

class GoodGrid
{
public:
	GoodGrid(Grid, float);

	std::vector< std::vector< int > > data;

	float width;
	float height;

	int horizontal_cell_count;
	int vertical_cell_count;

	float connectivity_distance;
};
