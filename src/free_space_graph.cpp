// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated May 7 2017 by Trent Ziemer

#include <wmp/free_space_graph.h>
#include <math.h>

FreeSpaceGraph::FreeSpaceGraph(GoodGrid * grid, int cells_per_row, int cells_per_column)
{
	int grid_index_x;
	int grid_index_y;
	float position_x;
	float position_y;
	int node_master_id = 0;
	
	gridPtr = grid;
	occupancy_threshold = 1; // 1 to 100

	bool random = false;
	if(random)
	{
		srand(time(NULL));
		// 5000 takes a while but is pretty good
		for(int i = 0; i < 5000 ; i++)
		{
			grid_index_x = rand() % grid->vertical_cell_count;
			grid_index_y = rand() % grid->horizontal_cell_count;

			position_x = ( (grid_index_x * ((float)cells_per_column / (float)grid->vertical_cell_count) * grid->height)/ ((float)cells_per_column - 1) ) - (grid->height/2);
			position_y = ( (grid_index_y * ((float)cells_per_row / (float)grid->horizontal_cell_count) * grid->width)/ ((float)cells_per_row - 1) ) - (grid->width/2);

			if(grid->data[grid_index_x][grid_index_y] < occupancy_threshold)
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
			grid_index_x = static_cast<int>(round(i * grid->vertical_cell_count  / cells_per_column));
			for(int j = 0; j <  cells_per_row; j++)
			{
				grid_index_y = static_cast<int>(round(j * grid->horizontal_cell_count / cells_per_row));

				position_x = ( (i * grid->height)/ ((float)cells_per_column - 1) ) - (grid->height/2);
				position_y = ( (j * grid->width)/ ((float)cells_per_row - 1) ) - (grid->width/2);

				if(grid->data[grid_index_x][grid_index_y] < occupancy_threshold)
				{
					nodeList.push_back(GraphNode(grid_index_x, grid_index_y, position_x, position_y, node_master_id));
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
		//ROS_INFO("Node %d at (%f, %f) is connected to:", i, nodeList[i].point.x, nodeList[i].point.y);
		for(int j = 0; j < nodeList[i].nearbyNodes.size(); j++)
		{
			//ROS_INFO("--- #%d node %d at (%f, %f)", j, nodeList[i].nearbyNodes[j].distantNode->id, nodeList[i].nearbyNodes[j].distantNode->point.x, nodeList[i].nearbyNodes[j].distantNode->point.y);
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
				return -1;
			}
		}

		return distance;
	}
}

// CAREFUL NEED THIS!
bool GraphNode::addEdge(GraphNode * distantNode, float weight)
{
	//ROS_INFO("Adding edge between (%f, %f) and (%f, %f): d = %f or %f.", x, y, distantNode->x, distantNode->y, sqrt(pow((distantNode->x - x),2) + pow((distantNode->y - y),2)), weight);
	nearbyNodes.push_back(GraphEdge(distantNode, weight));
	return true;
}

// Found online!
bool isIntersecting(GraphNode p1, GraphNode p2, GraphNode q1, GraphNode q2) {
    return (((q1.x-p1.x)*(p2.y-p1.y) - (q1.y-p1.y)*(p2.x-p1.x))
            * ((q2.x-p1.x)*(p2.y-p1.y) - (q2.y-p1.y)*(p2.x-p1.x)) < 0)
            &&
           (((p1.x-q1.x)*(q2.y-q1.y) - (p1.y-q1.y)*(q2.x-q1.x))
            * ((p2.x-q1.x)*(q2.y-q1.y) - (p2.y-q1.y)*(q2.x-q1.x)) < 0);
}