// An actually Good Grid (GoodGrid) object for WMP
// Created April 16 2017 by Trent Ziemer
// Last updated April 26 2017 by Trent Ziemer

#include <wmp/grid.h>

GoodGrid::GoodGrid(Grid occupancy_grid)
{
	std::vector<int> row_vector;
	for(int i = 0; i < occupancy_grid.data.size(); i++)
	{
		
		row_vector.push_back(occupancy_grid.data[i]);
		if( ((i + 1) % occupancy_grid.info.width) == 0)
		{
			data.push_back(row_vector);
			row_vector.clear();
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
	width = 5;
	height = 5;


}