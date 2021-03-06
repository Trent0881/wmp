// Path planning (using A star) object library definitions for WMP
// Created April 27 2017 by Trent Ziemer
// Last updated May 7 2017 by Trent Ziemer

#include <wmp/path_searcher.h>

// Various helpter function for the path searcher class to use for A* algorithm
bool addNodeToList(std::vector<GraphNode> * graph_node_list, GraphNode new_node, GoodGrid * grid)
{
	float node_distance;
	int occupancy_threshold = 1;
	unsigned int edges_added = 0;
	// Add the start point to the node list graph with some connectivity distance and collision occupancy threshold to other nodes
	for(int i = 0; i < (*graph_node_list).size(); i++)
	{
		node_distance = (*graph_node_list)[i].checkConnectivity(new_node, grid, grid->connectivity_distance, occupancy_threshold);
		if(node_distance != -1)
		{
			//ROS_INFO("Adding edge between (%f, %f) and (%f, %f): d = %f.", new_node.point.x, new_node.point.y, (*graph_node_list)[i].point.x, (*graph_node_list)[i].point.y, node_distance);
	
			(*graph_node_list)[i].addEdge(&new_node, node_distance);
			// Must add reciprocally here, but not for regular nodes!
			new_node.addEdge(&((*graph_node_list)[i]), node_distance);
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

bool listHasIdentifier(int index, std::vector<unsigned int> * list)
{
	for(int i = 0; i < list->size(); i++)
	{
		if(index == (*list)[i])
		{
			return true;
		}
	}
	return false;
}

bool removeFromVectorByValue(std::vector<unsigned int> * vector, int value)
{
	for(int i = 0; i < vector->size(); i++)
	{
		if( (*vector)[i] == value)
		{
			//ROS_INFO("Deleting element %d which is %d in list vector of size %lu!", i, (*vector)[i], vector->size());
			vector->erase(vector->begin() + i);
			return true;
		}
	}
	return false;
}

int findLowestScoreNode(std::vector<GraphNode> * list_of_all_nodes, std::vector<unsigned int> * list_of_indices)
{
	float current_f_score;
	// Assumes that at least one f score is less than INFINITY_APPROX, and to be fair they all should be
	float min_f_score = INFINITY_APPROX + 1;
	int min_f_score_index = -1;
	for(int i = 0; i < list_of_indices->size(); i++)
	{
		current_f_score = (*list_of_all_nodes)[ ((*list_of_indices)[i]) ].fScore;
		if( current_f_score < min_f_score)
		{
			min_f_score = current_f_score;
			min_f_score_index = (*list_of_indices)[i];
		}
	}

	if(min_f_score_index < 0)
	{
		ROS_WARN("COULD NOT FIND min f score (b/c index < -1)!");
	}
	return (*list_of_all_nodes)[min_f_score_index].id;
}

PathSearcher::PathSearcher(std::vector<GraphNode> graph, Point start_point, Point end_point, GoodGrid * grid, int cells)
{
	// Start by adding start and end points (as GraphNode's) to our graph, the node list
	int cells_per_column = cells;
	int cells_per_row = cells;
	
	int x_index, y_index;
	
	x_index = ( ( start_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( start_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );

	GraphNode start_node(x_index, y_index, start_point.x, start_point.y, graph.size());

	x_index = ( ( end_point.x + (grid->height/2))*((float)cells_per_column - 1)/ (grid->height) );
	y_index = ( ( end_point.y + (grid->width/2))*((float)cells_per_row - 1)/ (grid->width) );

	GraphNode end_node(x_index, y_index, end_point.x, end_point.y, graph.size() + 1);

	start_node.gScore = 0;
	start_node.fScore = nodeDistance(start_node, end_node);

	if(!addNodeToList(&graph, start_node, grid))
	{
		ROS_WARN("Failure to add start node: fix by adding an expanding distance!");
	}

	GraphNode* begin_node = &graph.back();
	int begin_node_id = graph.back().id;

	if(!addNodeToList(&graph, end_node, grid))
	{
		ROS_WARN("Failure to add end node: fix by adding an expanding distance!");
	}

	GraphNode* goal_node = &graph.back();

	// Now search graph for path from start to end using A*!
	// Add start identification number to the list of unexplored nodes
	openSet.push_back( begin_node_id );

	int current_index;
	int neighbor_index;

	float tentative_gScore;

	while(openSet.size() > 0)
	{
		current_index = findLowestScoreNode(&graph, &openSet);

		if(graph[current_index].id == goal_node->id)
		{
			// Rebuild path because we are done, and make a point cloud based on it, too
			while(current_index != begin_node_id)
			{
				finalPath.push_back(current_index);
				current_index = graph[current_index].cameFrom;
				graph[current_index].point.z = 0.03; // Just so it shows up above the rest of the map and graph stuff in Rviz
				pathCloud.push_back(graph[current_index].point);
			}
			break;
		}
		else
		{
			if(!removeFromVectorByValue(&openSet, current_index))
			{
				ROS_WARN("Couldn't remove value at some index from given vector set!");
			}

			closedSet.push_back(current_index);
		}

		for(int i = 0; i < graph[current_index].nearbyNodes.size(); i++)
		{
			//ROS_INFO("Current node's neighbor #%d has id #%d", i, graph[current_index].nearbyNodes[i].distantNode->id);
			neighbor_index = graph[current_index].nearbyNodes[i].distantNode->id;
			if(listHasIdentifier(neighbor_index, &closedSet))
			{
				continue;
			}

			tentative_gScore = graph[current_index].gScore + nodeDistance(graph[neighbor_index], *goal_node);

			if(!listHasIdentifier(neighbor_index, &openSet))
			{
				openSet.push_back(neighbor_index);
			}
			else if (tentative_gScore >= graph[neighbor_index].gScore)
			{
				continue;
			}

			// Then this is best path so far!
			graph[neighbor_index].cameFrom = current_index;
			graph[neighbor_index].gScore = tentative_gScore;
			graph[neighbor_index].fScore = tentative_gScore + nodeDistance(graph[neighbor_index], *goal_node);
		}
	}
}