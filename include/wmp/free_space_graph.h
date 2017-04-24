// Free Space Graph object header for WMP
// Created April 23 2017 by Trent Ziemer
// Last updated April 24 2017 by Trent Ziemer

#include <wmp/common.h>

class GraphNode
{
public:
	GraphNode(float, float);
	
	float distanceTo(*GraphNode);
	bool addEdge(*GraphNode, float);
	float x;
	float y;

private:

};

class FreeSpaceGraph
{
public:
	FreeSpaceGraph(Grid, int);
	bool connectNodes();
	std::vector<GraphNode> getNodes();

private:

	std::vector<GraphNode> nodeList;
	int num_of_nodes;
	int occupancy_threshold;
};
