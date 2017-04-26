// An actually Good Grid (GoodGrid) object for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated April 26 2017 by Trent Ziemer

#include <wmp/common.h>

class GoodGrid
{
public:

	GoodGrid(Grid);


	std::vector< std::vector<int> > grid;

	int width;
	int height;

	int horizontal_points;
	int vertical_points;

	float horizontal_resolution;
	float vertical_resolution;



private:
};
