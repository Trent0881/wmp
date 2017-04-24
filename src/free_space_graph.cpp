// Free Space Graph object definitions
// Created April 23 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/free_space_graph.h>

FreeSpaceGraph::FreeSpaceGraph(Grid occupancy_grid, int num_of_nodes)
{
	ROS_INFO("Width = %d", occupancy_grid.info.width);
	ROS_INFO("Height = %d", occupancy_grid.info.height);
	
	int grid_width = occupancy_grid.info.width;
	int grid_height = occupancy_grid.info.height;

	int grid_cell_x;
	int grid_cell_y;

	int grid_index_x;
	int grid_index_y;

	bool collision = false;

	occupancy_threshold = 1;
	srand(time(NULL));
	
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
					//ROS_INFO("Collision found: x = %d, y = %d", grid_index_x, grid_index_y);
					//ROS_INFO("...given points %d and %d!", grid_cell_x, grid_cell_y);
				}
			}
		}
		if (collision == false)
		{
			nodeList.push_back(GraphNode((float)grid_cell_x, (float)grid_cell_y));
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

GraphNode::GraphNode(float x_pos, float y_pos)
{
	x = x_pos;
	y = y_pos;
}