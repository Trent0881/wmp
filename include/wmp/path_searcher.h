// Path planning (using Dijkstras Best First Search) header for WMP
// Created April 27 2017 by Trent Ziemer
// Last updated May 3 2017 by Trent Ziemer

#include <wmp/common.h>
#include <wmp/free_space_graph.h>

class PathSearcher
{
public:

	PathSearcher(std::vector<GraphNode>, Point, Point, GoodGrid*);

private:

	std::vector<GraphNode*> closedSet;
	std::vector<GraphNode*> openSet;
	GraphNode cameFrom(float, float);
	std::vector<unsigned int>gScore;
	std::vector<unsigned int>fScore;
};
