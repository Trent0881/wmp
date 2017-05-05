// Path planning (using A star) object library definitions for WMP
// Created April 27 2017 by Trent Ziemer
// Last updated May 4 2017 by Trent Ziemer

#include <wmp/path_searcher.h>

bool addNodeToList(std::vector<GraphNode> * graph_node_list, GraphNode new_node, GoodGrid * grid)
{
	float node_distance;
	int occupancy_threshold = 1;
	float connectivity_distance = 0.3; // Can change whenever, I guess

	unsigned int edges_added = 0;
	// Add the start point to the node list graph with some connectivity distance and collision occupancy threshold to other nodes
	for(int i = 0; i < (*graph_node_list).size(); i++)
	{
		node_distance = (*graph_node_list)[i].checkConnectivity(new_node, grid, connectivity_distance, occupancy_threshold);
		if(node_distance != -1)
		{
			//ROS_INFO("Adding edge between start @ (%f, %f) and (%f, %f): d = %f.", new_node.point.x, new_node.point.y, (*graph_node_list)[i].point.x, (*graph_node_list)[i].point.y, node_distance);
	
			(*graph_node_list)[i].addEdge(&new_node, node_distance);
			// Must add reciprocally here, but not for regular nodes!
			new_node.addEdge(&((*graph_node_list)[i]), node_distance);
			edges_added++;
		}
	}
	ROS_INFO("edges added = %d", edges_added);
	ROS_INFO("new_node nearby nodes = %lu", new_node.nearbyNodes.size());
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

bool removeNodeByID(std::vector<GraphNode*> * graph, int identifier)
{
	for(int i = 0; i < graph->size(); i++)
	{
		if( (*graph)[i]->id == identifier)
		{
			ROS_INFO("Deleting element %d in list vector of size %lu!", i, graph->size());
			graph->erase( graph->begin() + i );
			return true;
		}
	}
	return false;
}

GraphNode* findLowestScoreNode(std::vector<GraphNode*> * graph)
{
	float current_f_score;
	// Assumes that at least one f score is less than INFINITY_APPROX, and to be fair they all should be
	float min_f_score = INFINITY_APPROX + 1;
	int min_f_score_index = -1;
	for(int i = 0; i < graph->size(); i++)
	{
		ROS_INFO("graph size = %lu", graph->size());
		current_f_score = (*graph)[i]->fScore;
		if( current_f_score < min_f_score)
		{
			min_f_score = current_f_score;
			min_f_score_index = i;
		}
	}
	if(min_f_score_index == -1)
	{
		ROS_WARN("COULD NOT FIND min f score!");
	}
	ROS_INFO("RETURNING, min_f_score_index = %d, min_f_score = %f", min_f_score_index, min_f_score);
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

	GraphNode end_node(x_index, y_index, end_point.x, end_point.y, graph.size() + 1);

	start_node.gScore = 0;
	start_node.fScore = nodeDistance(start_node, end_node);
	
	ROS_INFO("Graph size = %lu, first element neighbor count = %lu", graph.size(), graph.back().nearbyNodes.size());

	if(!addNodeToList(&graph, start_node, grid))
	{
		ROS_WARN("Failure to add start node: fix by adding an expanding distance!");
	}

	GraphNode* begin_node = &graph.back();

	ROS_INFO("Graph size = %lu, last element neighbor count = %lu", graph.size(), graph.back().nearbyNodes.size());

	if(!addNodeToList(&graph, end_node, grid))
	{
		ROS_WARN("Failure to add end node: fix by adding an expanding distance!");
	}

	GraphNode* goal_node = &graph.back();

	ROS_INFO("Graph size = %lu, last element neighbor count = %lu", graph.size(), graph.back().nearbyNodes.size());

	// Now search graph for path from start to end!!!!!

	// Add start node to the list of unexplored nodes
	openSet.push_back( begin_node );

	GraphNode * current;
	GraphNode * neighbor;

	float tentative_gScore;

	while(openSet.size() > 0)
	{
		current = findLowestScoreNode(&openSet);

		ROS_INFO("id = %d, x = %f, y = %f, size(nearbyNodes) = %lu", current->id, current->point.x, current->point.y, current->nearbyNodes.size());
		
		ROS_INFO("graph[421].id = %d (421); current->nearbyNodes.size() = %lu (8)", graph[421].id, current->nearbyNodes.size());
		ROS_INFO("graph[421].nearbyNodes[0].weight = %f (?); graph[421].nearbyNodes[0].distantNode->id = %d (?) ; graph[421].nearbyNodes[0].point.x = %f (?)", graph[421].nearbyNodes[0].weight, graph[421].nearbyNodes[0].distantNode->id, graph[421].nearbyNodes[0].distantNode->point.x);
		
		if(current->id == goal_node->id)
		{
			ROS_INFO("SUCCESS in ID MATCHING; current == goal is true");
			// REC PATH HERE!
		}
		else
		{
			ROS_INFO("Not yet at goal node");
			if(!removeNodeByID(&openSet, current->id))
			{
				ROS_WARN("COULD NOT FIND NODE ID %d TO REMOVE FROM GRAPH", current->id);
			}
			closedSet.push_back(current);
			ROS_INFO("Added current node to closed set");
		}

		for(int i = 0; i < current->nearbyNodes.size(); i++)
		{
			ROS_INFO("Current node's neighbor #%d has id #%d", i, current->nearbyNodes[i].distantNode->id);
			neighbor = current->nearbyNodes[i].distantNode;
			ROS_INFO("Analyzing node neighbor #%d", neighbor->id);
			if(nodeInList(neighbor, &closedSet))
			{
				ROS_INFO("...node was in closed set list");
				continue;
			}

			tentative_gScore = current->gScore + nodeDistance(*neighbor, *goal_node);
			ROS_INFO("tentative_gScore of %f found for this neigbor", tentative_gScore);
			ROS_INFO("Checking if neighbor is in open set");
			if(!nodeInList(neighbor, &openSet))
			{
				ROS_INFO("Adding to open set because was not in it");
				openSet.push_back(neighbor);
			}
			else if (tentative_gScore >= neighbor->gScore)
			{
				ROS_INFO("Continuing because node was in list, but tentative_gScore >= neighbor G Score");
				continue;
			}
			ROS_INFO("Neighbor node was in list, but had better g score than tentative_gScore, so good path, updating g scores!");
			// Then this is best path!
			//cameFrom[neighbor] = current
			neighbor->gScore = tentative_gScore;
			neighbor->gScore = tentative_gScore + nodeDistance(*neighbor, *goal_node);
			ROS_INFO("done with a single current node neighbor!");
		}
	ROS_INFO("Done with each current node neighbor!");
	}
}