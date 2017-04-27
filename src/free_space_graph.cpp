// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated April 26 2017 by Trent Ziemer

#include <wmp/free_space_graph.h>
#include <math.h>

FreeSpaceGraph::FreeSpaceGraph(GoodGrid * grid, int cells_per_row, int cells_per_column)
{
	int grid_index_x;
	int grid_index_y;
	occupancy_threshold = 1; // 1 to 100
	float position_x;
	float position_y;
	bool random = false;
	int node_master_id = 0;
	srand(time(NULL));
	gridPtr = grid;

	if(random == true)
	{
		for(int i = 0; i < grid->horizontal_cell_count * grid->vertical_cell_count; i++)
		{
			grid_index_x = rand() % grid->horizontal_cell_count;
			grid_index_y = rand() % grid->vertical_cell_count;
			if(grid->data[grid_index_x][grid_index_y] > occupancy_threshold)
			{
				// position_x = ?
				nodeList.push_back(GraphNode(grid_index_x, grid_index_y, position_x, position_y, node_master_id));
				node_master_id++;
			}	
		}
	}
	else
	{
		for(int i = 0; i < cells_per_column; i++)
		{
			grid_index_x = static_cast<int>(round(i * grid->vertical_cell_count  / cells_per_column)); /// grid->horizontal_resolution);
			for(int j = 0; j <  cells_per_row; j++)
			{
				grid_index_y = static_cast<int>(round(j * grid->horizontal_cell_count / cells_per_row));

				position_x = (i * grid->height)/ ((float)cells_per_column) - (grid->height/2);
				position_y = (j * grid->width)/ ((float)cells_per_row) - (grid->width/2);

				if(grid->data[grid_index_x][grid_index_y] < occupancy_threshold)
				{
					//ROS_INFO("Adding node @ [%d, %d] pos = (%f, %f).", i, j, position_x, position_y);
					//ROS_INFO("with i = %d, j = %d, hcc = %d, vcc = %d, ", i , j, grid->horizontal_cell_count, grid->vertical_cell_count);
					//ROS_INFO("and cpr = %d, cpc = %d, gix = %d, giy = %d   ", cells_per_row, cells_per_column, grid_index_x, grid_index_y);
					nodeList.push_back(GraphNode(i, j, position_x, position_y, node_master_id));
					
					node_master_id++;
				}
				else
				{
					//ROS_INFO("Cant add node @ [%d, %d] pos = (%f, %f) due to obstacle collision", i, j, position_x, position_y);
				}
			}
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
				distanceHeuristic = nodeList[i].checkConnectivity(nodeList[j], gridPtr, connectivity_distance, occupancy_threshold);
				
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

GraphNode::GraphNode(int i, int j, float x_pos, float y_pos, int id_number)
{
	x = i;
	y = j;
	point = Point(x_pos, y_pos, 0);
	id = id_number;
}

GraphNode::GraphNode(float x_pos, float y_pos)
{
	x = x_pos;
	y = y_pos;
}

// Check distance to another node
float GraphNode::checkConnectivity(GraphNode distantNode, GoodGrid * gridPtr, float connectivity_distance, int connectivity_threshold)
{
	float distance = sqrt(pow((distantNode.point.x - point.x),2) + pow((distantNode.point.y - point.y),2));
	//ROS_INFO(%f, )
	if(distance > connectivity_distance)
	{
		// Preemptive failure, due to too far away. The integer "-1" is the error code for this case.
		return -1;
	}
	else if(distance == 0)
	{
		ROS_WARN("WHY ARE YOU COMPUTING A DISTANCE OF ZERO?");
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

		int a;
		int b;

		PointCloud lineCloud = generateCloudLine((float)x * (gridPtr->vertical_cell_count/20), (float)y * (gridPtr->horizontal_cell_count/20), (float)distantNode.x, (float)distantNode.y);
		for(int i = 0; i < lineCloud.size(); i++)
		{
			a = static_cast<int>(round(lineCloud[i].x));
			b = static_cast<int>(round(lineCloud[i].y));
			//ROS_INFO("Line cloud pt (%f, %f); accessing (%d, %d)", lineCloud[i].x, lineCloud[i].y, a ,b);
			if(gridPtr->data[a][b] > connectivity_threshold)
			{
				ROS_INFO("Obstacle at (%d %d)", a, b);
				g_bad_nodes.push_back(Point(x/2.5 - 2.5, y/2.5 - 2.5, 0)); ///???
				g_bad_nodes_two.push_back(Point(distantNode.x/2.5 - 2.5, distantNode.y/2.5 - 2.5, 0)); ///???
				return -1;
			}
			else
			{
				//ROS_INFO("Threshhold is apparently good with value (%d) less than %d", gridPtr->data[a][b], connectivity_threshold);
			}
		}

		for(int i = 0; i < gridPtr->data.size(); i++)
		{
			for(int j = 0; j < gridPtr->data[i].size(); j++)
			{
				if(gridPtr->data[i][j] > 0)
				{
					//ROS_INFO("gridPtr->data[%d][%d]: %d", i, j, gridPtr->data[i][j]);
					if(i == a && j == b)
					{
						ROS_WARN("LIAR");
					}
				}
			}
		}

		//ROS_INFO("Path of distance %f found!", distance);
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