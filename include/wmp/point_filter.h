// Point Filter object header for WMP
// Created April 14 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/common.h>

class PointFilter
{
public:
	PointFilter();
	bool setPoint(Point new_point);
	bool translateAndShrink();
	bool cropPoint();
	Point getPoint(); 

private:

	float x_min;
	float x_max;
	float y_min;
	float y_max;
	float z_min;
	float z_max;

	Point point;
	Point center_point;
	bool first_time;
	float scale_factor;
};
