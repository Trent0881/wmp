// Free Space Graph object header for WMP
// Created April 23 2017 by Trent Ziemer
// Last updated April 24 2017 by Trent Ziemer

#include <wmp/common.h>

class GraphNode;

class GraphEdge
{
public:

	GraphEdge(GraphNode*, float);
	
private:
	GraphNode * distantNode;
	float weight;
};

GraphEdge::GraphEdge(GraphNode * distantNodePtr, float distance_weight)
{
	distantNode = distantNodePtr;
	weight = distance_weight;
}

class GraphNode
{
public:
	GraphNode(float, float);
	
	float distanceTo(GraphNode);
	bool addEdge(GraphNode*, float);
	float x;
	float y;

	float checkConnectivity(GraphNode, float);

private:
	std::vector<GraphEdge> nearbyNodes;
};

class FreeSpaceGraph
{
public:
	FreeSpaceGraph(Grid, int);
	bool connectNodes(float);
	std::vector<GraphNode> getNodes();
	PointCloud getNodesAsPointCloud();

private:

	std::vector<GraphNode> nodeList;
	int num_of_nodes;
	int occupancy_threshold;

	float x_offset;
	float y_offset;
	float grid_resolution;
};

// Forward function declaration for a mathematical operation that we need
bool doIntersect(GraphNode, GraphNode, GraphNode, GraphNode);
