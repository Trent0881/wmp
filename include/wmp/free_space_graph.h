// Free Space Graph object header for WMP
// Created April 23 2017 by Trent Ziemer
// Last updated April 24 2017 by Trent Ziemer

#include <wmp/common.h>
#include <wmp/grid.h>

#define INFINITY_APPROX 999999

// Such bad style!
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
	// grid_index_x, grid_index_y, position_x, position_y, and node_master_id
	GraphNode(int, int, float, float, int);
	// Degen node, not in actual graph list of nodes. Used for testing.
	GraphNode(float, float);
	float distanceTo(GraphNode);
	bool addEdge(GraphNode*, float);
	int x;
	int y;
	Point point;

	float checkConnectivity(GraphNode, GoodGrid*, float, int);
	std::vector<GraphEdge> nearbyNodes;

	int id;

	float gScore;
	float fScore;
	int cameFrom;
};

class FreeSpaceGraph
{
public:
	FreeSpaceGraph(GoodGrid *, int);
	FreeSpaceGraph(GoodGrid *, int, int);

	bool connectNodes(float);
	std::vector<GraphNode> getNodes();
	PointCloud getPointCloud();
	PointCloud createEdgeCloud();

	std::vector<GraphNode> nodeList;

private:

	GoodGrid * gridPtr;
	int num_of_nodes;
	int occupancy_threshold;

	float x_offset;
	float y_offset;
	float grid_resolution;
};

bool isIntersecting(GraphNode, GraphNode, GraphNode, GraphNode);
