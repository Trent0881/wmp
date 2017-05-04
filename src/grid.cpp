// An actually Good Grid (GoodGrid) object for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated May 3 2017 by Trent Ziemer

#include <wmp/grid.h>

GoodGrid::GoodGrid(Grid occupancy_grid)
{
	std::vector<int> row_vector;
	int j = 0;
	int index = 0;
	for(int i = 0; i < occupancy_grid.data.size(); i++)
	{
		//ROS_INFO("Trying to get data at %d, with i = %d, j = %d", index*occupancy_grid.info.width + j, index, j);

		row_vector.push_back(occupancy_grid.data[index*occupancy_grid.info.width + j]);
		if( ((index + 1) % occupancy_grid.info.height) == 0)
		{
			j++;
			data.push_back(row_vector);
			row_vector.clear();
			index = 0;
		}
		else
		{
			index++;
		}
	}

	assert(data.size() == occupancy_grid.info.height);
	
	for(int i = 0; i < data.size(); i++)
	{
		assert(data[i].size() == occupancy_grid.info.width);
	}

	for(int i = 0; i < occupancy_grid.info.height; i++)
	{
		for(int j = 0; j < occupancy_grid.info.width; j++)
		{
			//std::cout << data[i][j] << " ";
		}
		//std::cout << std::endl;
	}

	horizontal_cell_count = occupancy_grid.info.width;
	vertical_cell_count = occupancy_grid.info.height;

	// Should be from a calc on the above kinda stuff, passed in from occ grid
	width = 5;
	height = 5;
}