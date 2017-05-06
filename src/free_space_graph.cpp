// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated April 27 2017 by Trent Ziemer

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

				position_x = ( (i * grid->height)/ ((float)cells_per_column - 1) ) - (grid->height/2);
				position_y = ( (j * grid->width)/ ((float)cells_per_row - 1) ) - (grid->width/2);

				position_x -= (0.5) * (5/cells_per_column);
				position_y += (5*0.5) * (5/cells_per_row);

				if(grid->data[grid_index_x][grid_index_y] < occupancy_threshold)
				{
					//ROS_INFO("Adding node @ [%d, %d] pos = (%f, %f).", i, j, position_x, position_y);
					//ROS_INFO("with i = %d, j = %d, hcc = %d, vcc = %d, ", i , j, grid->horizontal_cell_count, grid->vertical_cell_count);
					//ROS_INFO("and cpr = %d, cpc = %d, gix = %d, giy = %d   ", cells_per_row, cells_per_column, grid_index_x, grid_index_y);
					nodeList.push_back(GraphNode(grid_index_x, grid_index_y, 1, 1, node_master_id));
					node_master_id++;
				}
			}
		}
	}
}

std::vector<GraphNode> FreeSpaceGraph::getNodes()
{
	return nodeList;
}

PointCloud FreeSpaceGraph::getPointCloud()
{
	PointCloud graphPointCloud;

	for(int i = 0; i < nodeList.size(); i++)
	{
		graphPointCloud.push_back(nodeList[i].point);
	}

	return graphPointCloud;
}

bool FreeSpaceGraph::connectNodes(float connectivity_distance)
{
	float node_distance;

	for(int i = 0; i < nodeList.size(); i++)
	{
		for(int j = 0; j < nodeList.size(); j++)
		{
			if(i != j)
			{
				// For any two distinct nodes in our area with indices i and j...
				//ROS_INFO("Conn dist = %f, grid res = %f, cd/gs = %f", connectivity_distance, grid_resolution, connectivity_distance/grid_resolution);
				//ROS_INFO("Checking conn btwn: (");
				node_distance = nodeList[i].checkConnectivity(nodeList[j], gridPtr, connectivity_distance, occupancy_threshold);
				
				// If the nodes have a positive distance associated between them because they are within the connectivity distance...
				if(node_distance != -1)
				{		
					// Add an edge to the list of vertex nodes on the graph, with weighting equal to that distance	
					nodeList[i].addEdge(&nodeList[j], node_distance);
				}
			}
		}
	}
}

PointCloud FreeSpaceGraph::createEdgeCloud()
{
	PointCloud graph_edge_clouds;
	for(int i = 0; i < nodeList.size(); i++)
	{
		//ROS_INFO("Node %d [%f, %f] at (%f, %f) is connected to:", i, planningGraph.nodeList[i].x, planningGraph.nodeList[i].y, planningGraph.nodeList[i].point.x, planningGraph.nodeList[i].point.y);
		for(int j = 0; j < nodeList[i].nearbyNodes.size(); j++)
		{
			//ROS_INFO("--- #%d node %d at (%f, %f)", j, planningGraph.nodeList[i].nearbyNodes[j].distantNode->id, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.x, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.y);
			
			PointCloud edge_cloud = generateCloudLine(nodeList[i].point.x, nodeList[i].point.y, nodeList[i].nearbyNodes[j].distantNode->point.x, nodeList[i].nearbyNodes[j].distantNode->point.y);
			
			for(int k = 0; k < edge_cloud.size(); k++)
			{
				graph_edge_clouds.push_back(edge_cloud[k]);
			}
		}
	}	
	return graph_edge_clouds;
}


GraphNode::GraphNode(int i, int j, float x_pos, float y_pos, int id_number)
{
	x = i;
	y = j;
	point = Point(x_pos, y_pos, 0);
	id = id_number;
	gScore = INFINITY_APPROX;
	fScore = INFINITY_APPROX;
}

GraphNode::GraphNode(float x_pos, float y_pos)
{
	x = x_pos;
	y = y_pos;
	gScore = INFINITY_APPROX;
	fScore = INFINITY_APPROX;
}

// Check distance to another node
float GraphNode::checkConnectivity(GraphNode distantNode, GoodGrid * gridPtr, float connectivity_distance, int connectivity_threshold)
{
	float distance = sqrt(pow((distantNode.point.x - point.x),2) + pow((distantNode.point.y - point.y),2));
	if(distance > connectivity_distance)
	{
		// Preemptive failure, due to too far away. The integer "-1" is the error code for this case.
		return -1;
	}
	else if(distance == 0)
	{
		//ROS_WARN("WHY ARE YOU COMPUTING A DISTANCE OF ZERO?");
	}
	else
	{
		int a;
		int b;

		PointCloud lineCloud = generateCloudLine((float)(x), (float)(y), (float)distantNode.x, (float)distantNode.y);
		for(int i = 0; i < lineCloud.size(); i++)
		{
			a = static_cast<int>(round(lineCloud[i].x));
			b = static_cast<int>(round(lineCloud[i].y));
			//ROS_INFO("Line cloud pt (%f, %f); accessing (%d, %d)", lineCloud[i].x, lineCloud[i].y, a ,b);
			if(gridPtr->data[a][b] > connectivity_threshold)
			{
				//ROS_INFO("Obstacle at [%d %d] @ (%f %f)", a, b, lineCloud[i].x,  lineCloud[i].y);
				//g_bad_nodes.push_back(point); ///???
				//g_bad_nodes_two.push_back(distantNode.point); ///???
				return -1;
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

bool isIntersecting(GraphNode p1, GraphNode p2, GraphNode q1, GraphNode q2) {
    return (((q1.x-p1.x)*(p2.y-p1.y) - (q1.y-p1.y)*(p2.x-p1.x))
            * ((q2.x-p1.x)*(p2.y-p1.y) - (q2.y-p1.y)*(p2.x-p1.x)) < 0)
            &&
           (((p1.x-q1.x)*(q2.y-q1.y) - (p1.y-q1.y)*(q2.x-q1.x))
            * ((p2.x-q1.x)*(q2.y-q1.y) - (p2.y-q1.y)*(q2.x-q1.x)) < 0);
}