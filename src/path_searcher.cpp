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

float nodeDistance(GraphNode a, GraphNode b)
{
	return sqrt(pow((a.point.x - b.point.x),2) + pow((a.point.y - b.point.y),2));
}

bool removeNodeByID(std::vector<GraphNode> * graph, int identifier)
{
	for(int i = 0; i < graph->size(); i++)
	{
		if( (*graph)[i].id == identifier)
		{
			graph->erase( graph->begin() + i );
			return true;
		}
	}
	return false;
}

GraphNode* findLowestScoreNode(std::vector<GraphNode*> * graph)
{
	unsigned int current_f_score;
	// Assumes that at least one f score is less than INFINITY_APPROX, and to be fair they all should be
	unsigned int min_f_score = INFINITY_APPROX;
	int min_f_score_index;
	for(int i = 0; i < graph->size(); i++)
	{
		current_f_score = (*graph)[i]->fScore;
		if( current_f_score < min_f_score)
		{
			min_f_score = current_f_score;
			min_f_score_index = i;
		}
	}
	return (*graph)[min_f_score_index];
}

bool nodeInList(GraphNode * node, std::vector<GraphNode*> * graph)
{
	for(int i = 0; i < graph->size(); i++)
	{
		if(node->id == (*graph)[i]->id)
		{
			return true;
		}
	}
	return false;
}

PathSearcher::PathSearcher(std::vector<GraphNode> graph, Point start_point, Point end_point, GoodGrid * grid)
{
	// Start by adding start and end points (as GraphNode's) to our graph, the node list
	int cells_per_column = 30;
	int cells_per_row = 30;
	
	int x_index, y_index;
	
	x_index = ( ( start_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( start_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );

	GraphNode start_node(x_index, y_index, start_point.x, start_point.y, graph.size());

	x_index = ( ( end_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( end_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );

	GraphNode end_node(x_index, y_index, end_point.x, end_point.y, graph.size());

	start_node.gScore = 0;
	start_node.fScore = nodeDistance(start_node, end_node);
	
	if(!addNodeToList(&graph, start_node, grid))
	{
		ROS_WARN("Failure to add start node: fix by adding an expanding distance!");
	}

	if(!addNodeToList(&graph, end_node, grid))
	{
		ROS_WARN("Failure to add end node: fix by adding an expanding distance!");
	}

	// Now search graph for path from start to end!!!!!

	// Add start node to the list of unexplored nodes
	openSet.push_back(&start_node);

	GraphNode * current;
	GraphNode * neighbor;

	int tentative_gScore;

	while(openSet.size() > 0)
	{
		current = findLowestScoreNode(&openSet);

		if(current->id == end_node.id)
		{
			ROS_INFO("SUCCESS in ID MATCHING; current == goal is true");
			// REC PATH HERE!
		}
		else
		{
			ROS_INFO("IDs dont match current and end, not yet at goal node");
			//if(!removeNodeByID(&graph, current->id))
			{
				//ROS_WARN("COULD NOT FIND NODE ID %d TO REMOVE FROM GRAPH", current->id);
			}
			//closedSet.push_back(current);
		}

		for(int i = 0; i < current->nearbyNodes.size(); i++)
		{
			neighbor = current->nearbyNodes[i].distantNode;
			if(nodeInList(neighbor, &closedSet))
			{
				continue;
			}
			
			tentative_gScore = current->gScore + nodeDistance(*neighbor, end_node);
			if(!nodeInList(neighbor, &openSet))
			{
				openSet.push_back(neighbor)
			}


		}


	}


}