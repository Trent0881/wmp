// Point Filter object for WMP
// Created April 14 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/point_filter.h>

PointFilter::PointFilter()
{
	first_time = true;
	scale_factor = 0.08;
	
	x_min = -2.5;
	x_max = 2.5;
	y_min = -2.5;
	y_max = 2.5;
	z_min = -0.3;
	z_max = 0.12;
}

bool PointFilter::setPoint(Point new_point)
{
	point = new_point;
	if(first_time == true)
	{
		first_time = false;
		// Transform the future points around this point
		center_point = point;
	}
}

bool PointFilter::translateAndShrink()
{
	// Translate to center the point around the center point, and shrink/expand by some constant factor for ease
	Point placeholder_point(point.x - center_point.x, point.y - center_point.y, point.z - center_point.z);
	point.x = (placeholder_point.x)*scale_factor;
	point.y = (placeholder_point.y)*scale_factor;
	point.z = (placeholder_point.z)*scale_factor;

	return true;
}

bool PointFilter::cropPoint()
{
	if(point.x > x_max || point.x < x_min ||
		point.y > y_max || point.y < y_min ||
		point.z > z_max || point.z < z_min)
	{
		return false;
	}
	return true;
}

Point PointFilter::getPoint()
{
	return point;
}