// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/free_space_graph.h>

FreeSpaceGraph::FreeSpaceGraph(GoodGrid * grid, int horizontal_cell_count, int vertical_cell_count)
{
	int grid_index_x;
	int grid_index_y;
	occupancy_threshold = 1; // 1 to 100

	bool random = false;
	int node_master_id = 0;
	srand(time(NULL));

	if(random == true)
	{
		grid_index_x = rand() % grid->horizontal_cell_count;
		grid_index_y = rand() % grid->vertical_cell_count;
		for(int i = 0; i < horizontal_cell_count*vertical_cell_count; i++)
		{
			if(grid->data[grid_index_x][grid_index_y] > occupancy_threshold)
			{
				x_position = 
				nodeList.push_back(GraphNode(grid_index_x, grid_index_y, x_position, y_position, node_master_id));
				node_master_id++;
			}	
		}
	}
	else
	{
		for(int i = 0; i < horizontal_cell_count; i++)
		{
			grid_index_x = static_cast<int>std::round(i / grid->horizontal_resolution);
			for(int j = 0; j < vertical_cell_count; j++)
			{
				grid_index_y = static_cast<int>std::round(j / grid->vertical_resolution);
				if(grid->data[grid_index_x][grid_index_y] > occupancy_threshold)
				{
					x_position = 
					nodeList.push_back(GraphNode(grid_index_x, grid_index_y, x_position, y_position, node_master_id));
					node_master_id++;
				}
			}
		}
	}

	grid_cell_y = 0;
	for(int i = 0; i < num_of_nodes; i++)
	{
		/* RANDOM!*/

		
		grid_index_y = rand() % grid->vertical_cell_count;
		
		/* UNIFORM! 
		grid_cell_x = (4*i) % occupancy_grid.info.width;
		if(grid_cell_x == 0)
			grid_cell_y = grid_cell_y + 4;

		//ROS_INFO("(%d and %d)", grid_cell_x, grid_cell_y);
		*/
		for(int j = 0; j < occupancy_grid.data.size(); j++)
		{
			if(occupancy_grid.data[j] >= occupancy_threshold)
			{
				grid_index_y = 0;
				int index = j;
				while(index >= grid->horizontal_cells)
				{
					index = index - grid->horizontal_cells;
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
			nodeList.push_back(GraphNode(grid_cell_x, grid_cell_y, x_offset, y_offset, grid_resolution, node_master_id));
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
				distanceHeuristic = nodeList[i].checkConnectivity(nodeList[j],  connectivity_distance, gridPtr);
				
				if(distanceHeuristic != -1)
				{			
					nodeList[i].addEdge(&nodeList[j], distanceHeuristic);
				}
				else
				{
				}
				// MORE???!??!
			}
		}
	}
}

GraphNode::GraphNode(float x_pos, float y_pos, float x_offset, float y_offset, float grid_resolution, int id_number)
{
	x = x_pos;
	y = y_pos;
	point = Point((x*grid_resolution + x_offset), (y*grid_resolution + y_offset), 0);
	id = id_number;
}

GraphNode::GraphNode(float x_pos, float y_pos)
{
	x = x_pos;
	y = y_pos;

}

// Check distance to another node
float GraphNode::checkConnectivity(GraphNode distantNode, float connectivity_distance, Grid * gridPtr)
{
	float distance = sqrt(pow((distantNode.x - x),2) + pow((distantNode.y - y),2));
	
	if(distance > connectivity_distance/gridPtr->info.resolution)
	{
		// Preemptive failure, due to too far away. The integer "-1" is the error code for this case.
		return -1;
	}
	else
	{
		if(x < 0 || y < 0 || distantNode.x < 0 ||distantNode.y < 0)
		{
			ROS_WARN("NEGGGG!");
			return -1;
		}
		if( (float)x > 200 ||  (float)y > 200 ||  (float)distantNode.x > 200 ||  (float)distantNode.y > 200)
		{
			ROS_WARN("TOOLARGE, PROBS. HARD CODED!");
			return -1;
		}
		
		// HERE1

		PointCloud lineCloud = generateCloudLine(x, y, distantNode.x, distantNode.y);
		for(int i = 0; i < lineCloud.size(); i++)
		{
			int a = lineCloud[i].x;
			int b = lineCloud[i].y;
			if(gridPtr->data[ a * gridPtr->info.width + b] > 1) //  CHANGE TO OCC THR VAR (FROM ELSEWHERE)
			{
				ROS_INFO("Obstacle at (%d %d)", a, b);
				g_bad_nodes.push_back(Point(x*gridPtr->info.resolution - 2.5, y*gridPtr->info.resolution - 2.5, 0));
				g_bad_nodes_two.push_back(Point(distantNode.x*gridPtr->info.resolution - 2.5, distantNode.y*gridPtr->info.resolution - 2.5, 0));
				return -1;
			}
		}

		// HERE2
		//ROS_INFO("Path of distance %f found!", distance);
		return distance*gridPtr->info.resolution;
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
float orientation(GraphNode p, GraphNode q, GraphNode r)
{
    // See http://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    float val = (q.y - p.y) * (r.x - q.x) -
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
    float o1 = orientation(p1, q1, p2);
    float o2 = orientation(p1, q1, q2);
    float o3 = orientation(p2, q2, p1);
    float o4 = orientation(p2, q2, q1);
 
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

bool isIntersecting(GraphNode p1, GraphNode p2, GraphNode q1, GraphNode q2) {
    return (((q1.x-p1.x)*(p2.y-p1.y) - (q1.y-p1.y)*(p2.x-p1.x))
            * ((q2.x-p1.x)*(p2.y-p1.y) - (q2.y-p1.y)*(p2.x-p1.x)) < 0)
            &&
           (((p1.x-q1.x)*(q2.y-q1.y) - (p1.y-q1.y)*(q2.x-q1.x))
            * ((p2.x-q1.x)*(q2.y-q1.y) - (p2.y-q1.y)*(q2.x-q1.x)) < 0);
}