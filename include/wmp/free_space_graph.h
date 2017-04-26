// Free Space Graph object header for WMP
// Created April 23 2017 by Trent Ziemer
// Last updated April 24 2017 by Trent Ziemer

#include <wmp/common.h>
#include <wmp/grid.h>

// Such bad style
PointCloud generateCloudLine(float x1, float y1, float x2, float y2);

// Globals for checking/testing
PointCloud g_bad_nodes;
PointCloud g_bad_nodes_two;

class GraphNode;

class GraphEdge
{
public:

	GraphEdge(GraphNode*, float);
		GraphNode * distantNode;
private:

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
	GraphNode(float, float, float, float, float, int);
	// Degen node, not in master list. Used for testing.
	GraphNode(float, float);
	float distanceTo(GraphNode);
	bool addEdge(GraphNode*, float);
	float x;
	float y;
	Point point;

	float checkConnectivity(GraphNode, float, Grid*);
	std::vector<GraphEdge> nearbyNodes;

	int id;

private:

};

class FreeSpaceGraph
{
public:
	FreeSpaceGraph(GoodGrid *, int);
	FreeSpaceGraph(GoodGrid *, int, int);
	bool connectNodes(float);
	std::vector<GraphNode> getNodes();
	PointCloud getNodesAsPointCloud();
	std::vector<GraphNode> nodeList;
private:

	GoodGrid * gridPtr;
	int num_of_nodes;
	int occupancy_threshold;

	float x_offset;
	float y_offset;
	float grid_resolution;
};

// Forward function declaration for a mathematical operation that we need
bool doIntersect(GraphNode, GraphNode, GraphNode, GraphNode);

bool isIntersecting(GraphNode, GraphNode, GraphNode, GraphNode);