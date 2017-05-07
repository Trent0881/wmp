// Path planning (A* (aka A-star) ) header for WMP
// Created April 27 2017 by Trent Ziemer
// Last updated May 5 2017 by Trent Ziemer

#include <wmp/common.h>
#include <wmp/free_space_graph.h>

class PathSearcher
{
public:

	PathSearcher(std::vector<GraphNode>, Point, Point, GoodGrid*, int);
	std::vector<unsigned int> finalPath;
	PointCloud pathCloud;

private:

	std::vector<unsigned int> closedSet;
	std::vector<unsigned int> openSet;
	std::vector<unsigned int> gScore;
	std::vector<unsigned int> fScore;
};
