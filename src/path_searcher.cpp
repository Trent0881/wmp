// Path planning (using A star) object library definitions for WMP
// Created April 27 2017 by Trent Ziemer
// Last updated May 3 2017 by Trent Ziemer

#include <wmp/path_searcher.h>

bool addNodeToList(std::vector<GraphNode> * graph_node_list, GraphNode new_node, GoodGrid * grid)
{
	float node_distance;
	int occupancy_threshold = 1;
	float connectivity_distance = 0.5; // Can change whenever, I guess

	unsigned int edges_added = 0;
	// Add the start point to the node list graph with some connectivity distance and collision occupancy threshold to other nodes
	for(int i = 0; i < (*graph_node_list).size(); i++)
	{
		node_distance = (*graph_node_list)[i].checkConnectivity(new_node, grid, connectivity_distance, occupancy_threshold);
		if(node_distance != -1)
		{
			//ROS_INFO("Adding edge between start @ (%f, %f) and (%f, %f): d = %f.", new_node.point.x, new_node.point.y, (*graph_node_list)[i].point.x, (*graph_node_list)[i].point.y, node_distance);
	
			(*graph_node_list)[i].addEdge(&new_node, node_distance);
			edges_added++;
		}
	}
	if(edges_added > 0)
	{
		graph_node_list->push_back(new_node);
		return true;
	}
	else
	{
		return false;
	}
}

PathSearcher::PathSearcher(std::vector<GraphNode> graph, Point start_point, Point end_point, GoodGrid * grid)
{
	// Start by adding start and end points (as GraphNode's) to our ...
	int cells_per_column = 30;
	int cells_per_row = 30;
	
	int x_index, y_index;
	
	x_index = ( ( start_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( start_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );
	
	ROS_INFO("Size of graph starts as: %lu", graph.size());
	if(!addNodeToList(&graph, GraphNode(x_index, y_index, start_point.x, start_point.y, 0), grid))
	{
		ROS_WARN("Failure to add start node: fix by adding an expanding distance!");
	}
	else
	{
		ROS_INFO("Size of graph is now: %lu", graph.size());
	}

	x_index = ( ( end_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( end_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );

	if(!addNodeToList(&graph, GraphNode(x_index, y_index, end_point.x, end_point.y, 0), grid))
	{
		ROS_WARN("Failure to add end node: fix by adding an expanding distance!");
	}
	else
	{
		ROS_INFO("Size of graph is now: %lu", graph.size());
	}


}