// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/free_space_graph.h>

FreeSpaceGraph::FreeSpaceGraph(Grid occupancy_grid, int num_of_nodes)
{
	int grid_width = occupancy_grid.info.width;
	int grid_height = occupancy_grid.info.height;

	x_offset = occupancy_grid.info.origin.position.x;
	y_offset = occupancy_grid.info.origin.position.y;
	grid_resolution = occupancy_grid.info.resolution;

	// occupancy_grid.info.origin.orientation.xyzw;

	int grid_cell_x;
	int grid_cell_y;

	int grid_index_x;
	int grid_index_y;

	bool collision = false;

	occupancy_threshold = 1; //  1 to 100
	srand(time(NULL));
	int node_master_id = 0;
	for(int i = 0; i < num_of_nodes; i++)
	{
		grid_cell_x = rand() % occupancy_grid.info.width;
		grid_cell_y = rand() % occupancy_grid.info.height;

		for(int j = 0; j < occupancy_grid.data.size(); j++)
		{
			if(occupancy_grid.data[j] >= occupancy_threshold)
			{
				grid_index_y = 0;
				int index = j;
				while(index >= grid_width)
				{
					index = index - grid_width;
					grid_index_y++;
				}
				grid_index_x = index;

				if((grid_cell_x == grid_index_x) && (grid_cell_y == grid_index_y))
				{
					collision = true;
					//ROS_INFO("Point-sampling obstacle-collision found: x = %d, y = %d", grid_index_x, grid_index_y);
					//ROS_INFO("...given points %d and %d!", grid_cell_x, grid_cell_y);
				}
			}
		}
		if (collision == false)
		{
			nodeList.push_back(GraphNode(grid_cell_x, grid_cell_y, node_master_id));
			node_master_id++;
		}
		else
		{
			collision = false;
		}
	}
}

std::vector<GraphNode> FreeSpaceGraph::getNodes()
{
	return nodeList;
}

PointCloud FreeSpaceGraph::getNodesAsPointCloud()
{
	PointCloud graphPointCloud;

	for(int i = 0; i < nodeList.size(); i++)
	{
		// Maybe with other changes: graphPointCloud.push_back(Point(nodeList[i].x*grid_resolution + x_offset, nodeList[i].y*grid_resolution + y_offset, 0));

		graphPointCloud.push_back(Point((nodeList[i].x - x_offset)/grid_resolution, (nodeList[i].y - y_offset)/grid_resolution, 0));
	}

	return graphPointCloud;
}

bool FreeSpaceGraph::connectNodes(float connectivity_distance)
{
	float distanceHeuristic;
	for(int i = 0; i < nodeList.size(); i++)
	{
		for(int j = 0; j < nodeList.size(); j++)
		{
			if(i != j)
			{
				//ROS_INFO("Conn dist = %f, grid res = %f, cd/gs = %f", connectivity_distance, grid_resolution, connectivity_distance/grid_resolution);
				// For any two distinct nodes in our area with labels i and j...
				distanceHeuristic = nodeList[i].checkConnectivity(nodeList[j], connectivity_distance/grid_resolution)*grid_resolution;
				
				if(distanceHeuristic != -1*grid_resolution)
				{			
					nodeList[i].addEdge(&nodeList[j], distanceHeuristic);
				}
				else
				{

				}

				// MORE!
			}
		}
	}
}

GraphNode::GraphNode(float x_pos, float y_pos, int id_number)
{
	x = x_pos;
	y = y_pos;
	//xm = x_pos*
	id = id_number;
}

GraphNode::GraphNode(float x_pos, float y_pos)
{
	x = x_pos;
	y = y_pos;
	//xm = x_pos*
}

// Check distance to another node
float GraphNode::checkConnectivity(GraphNode distantNode, float connectivity_distance)
{
	float distance = sqrt(pow((distantNode.x - x),2) + pow((distantNode.y - y),2));
	
	if(distance > connectivity_distance)
	{
		// Preemptive failure, due to too far away. The integer "-1" is the error code for this case.
		return -1;
	}
	else
	{
		// Check if there is an intersection between the line segment between these two nodes and any occupied cell on the grid
		// for all line segments in rectangular sub-grid.

		float lower_bound_x;
		float lower_bound_y;
		float upper_bound_x;
		float upper_bound_y;
		float delta_i;
		float delta_j;
		// TBD
		lower_bound_x = (x - 0.5);
		lower_bound_y = (y - 0.5);
		upper_bound_x = (distantNode.x - 0.5);
		upper_bound_y = (distantNode.y - 0.5);
		delta_i = 1;
		delta_j = 1;

		for(float i = lower_bound_x; i <= upper_bound_x; i = i + delta_i)
		{
			for(float j = lower_bound_y; i <= upper_bound_y; j = j + delta_j)
			{
				GraphNode P(i, j);
				GraphNode Q(i + 1, j);
				if(doIntersect(*this, distantNode, P, Q))
				{
					//ROS_INFO("Path (%f, %f) -> (%f, %f) intersects line segment (%f, %f) -> (%f, %f)", 
					//		x, y, distantNode.x, distantNode.y, P.x, P.y, Q.x, Q.y);
					return -1;
				}

				P.x = i + 1;
				P.y = j - 1;
				
				if(doIntersect(*this, distantNode, P, Q))
				{
					//ROS_INFO("Path (%f, %f) -> (%f, %f) intersects line segment (%f, %f) -> (%f, %f)", 
					//		x, y, distantNode.x, distantNode.y, P.x, P.y, Q.x, Q.y);
					return -1;
				}
			}
		}

		//ROS_INFO("Path of distance %f found!", distance);
		//ROS_INFO("THIS IS A PROBLEM");

		return distance;
	}
}

// CAREFUL NEED THIS!
bool GraphNode::addEdge(GraphNode * distantNode, float weight)
{
	//ROS_INFO("Adding edge between (%f, %f) and (%f, %f): d = %f or %f.", x, y, distantNode->x, distantNode->y, sqrt(pow((distantNode->x - x),2) + pow((distantNode->y - y),2)), weight);

	nearbyNodes.push_back(GraphEdge(distantNode, weight));
	//distantNode->addEdge ???
	return true;
}

// I DID NOT DESIGN OR WRITE THE FOLLOWING CODE
// All credit goes to unknown GeeksForGeeks.org writer
// http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// Modified slightly to fit my needs

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(GraphNode p, GraphNode q, GraphNode r)
{
    if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
        q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
       return true;
 
    return false;
}
 
// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(GraphNode p, GraphNode q, GraphNode r)
{
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
 
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(GraphNode p1, GraphNode q1, GraphNode p2, GraphNode q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}