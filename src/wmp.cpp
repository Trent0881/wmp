// Wobbler Motion Planning node
// Created April 14 2017 by Trent Ziemer
// Last updated May 5 2017 by Trent Ziemer

// WMP-specific custom libraries for object
#include <wmp/common.h>
#include <wmp/point_filter.h>
#include <wmp/cloud_compressor.h>
#include <wmp/path_searcher.h> //(which includes grid and free_space_graph)

// File IO for C++
#include <iostream>
#include <fstream>

// Point cloud of sensor measurements to be used to make our sensor model of parameters
PointCloud g_point_cloud_data;
PointCloud g_shrunk_cloud;
PointCloud g_filtered_cloud;

ros::NodeHandle * nh_ptr;

bool getDataFromFile(std::string filename)
{
    std::ifstream input_data;
    input_data.open(filename.c_str());
    if(!input_data.is_open())
    {
        ROS_WARN("%s IS NOT OPEN", filename.c_str());
    }

    bool getting_x = true;
    bool getting_y = false;
    bool getting_z = false;

    PointFilter pointFilter;
    float value;
    float x;
    float y;
    while(input_data >> value)
    {
        if(getting_x)
        {
        	x = value;
        	getting_x = false;
        	getting_y = true;
        }
        else if(getting_y)
        {
        	y = value;
        	getting_y = false;
        	getting_z = true;
        }
        else if(getting_z)
        {
        	//ROS_INFO("Read a Point(%f, %f, %f)", x, y, value);
        	pointFilter.setPoint(Point(x, y, value));
            if(pointFilter.translateAndShrink())
        	{
        		g_shrunk_cloud.push_back(pointFilter.getPoint());
			    if(pointFilter.cropPoint())
	        	{
					g_filtered_cloud.push_back(pointFilter.getPoint());
	        	}	
        	}

        	// and cycle state machine...
			getting_z = false;
        	getting_x = true;
        }
    }

    if(g_shrunk_cloud.points.size() > 0)
    {
    	return true;
    }
    else
    {
    	return false;
    }
}

// Builds and returns a point cloud in a line from the first coordinate x,y to the second coordinate x,y. z = 0.
PointCloud generateCloudLine(float x1, float y1, float x2, float y2)
{
	int number_of_points_per_line = 10;
	float delta_x = (x2 - x1)/number_of_points_per_line;
	float delta_y = (y2 - y1)/number_of_points_per_line;
	PointCloud line_cloud;
	for(int i = 0; i < number_of_points_per_line; i++)
	{
		line_cloud.push_back(Point(x1 + delta_x*i, y1 + delta_y*i, 0));
		//ROS_INFO("Connecting # %d: (%f, %f) to (%f, %f): (%f, %f)", i, x1, y1, x2, y2, x1 + delta_x*i, y1 + delta_y*i);
	}
	return line_cloud;
}

///////////////////////////        MAIN       ////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc,argv,"wobbler_motion_planning");

    ros::NodeHandle nh("~");
    nh_ptr = &nh;

    std::string filename;

	if(!nh_ptr->getParam("filename", filename))
	{
		ROS_WARN("Failed to get filename for input data source!");
	}

	if(!getDataFromFile(filename))
	{
		ROS_WARN("FAILED to get file data!");
	}

	CloudCompressor cloudCompressor(200, 200, -2.5, -2.5);
	
	if(!cloudCompressor.setCloud(g_filtered_cloud))
	{
		ROS_WARN("Failed to set cloud data!");
	}

	if(!cloudCompressor.compressFlat())
	{
		ROS_WARN("FAILED to compress cloud data!");
	}

	if(!cloudCompressor.compressToGrid())
	{
		ROS_WARN("FAILED to compress map cloud to grid!");
	}

	ROS_INFO("Building planning graph object");

	GoodGrid grid = GoodGrid(cloudCompressor.getGrid());

	int cells_per_side = 50;
	FreeSpaceGraph planningGraph(&grid, cells_per_side, cells_per_side);
	
	PointCloud sample_points = planningGraph.getPointCloud();

	ROS_INFO("Connecting nodes on graph");

	float connection_radius = (float)10/cells_per_side;
	planningGraph.connectNodes(connection_radius);

	ROS_INFO("Checking node consistency across edges!");

	std::vector<GraphNode> nodes = planningGraph.getNodes();

	ROS_INFO("Building node-based pathway planner");

	PathSearcher pathway(planningGraph.getNodes(), Point(0.3, 1.5, 0), Point(1.1, -1.1, 0), &grid); //Point(1.1, -1.1, 0)

	ROS_INFO("Creating graph edge clouds");

	PointCloud graph_edge_clouds = planningGraph.createEdgeCloud();

	ROS_INFO("Done with processing and planning! Now publishing all this data.");

	ros::Publisher input_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ipc", 1);
	ros::Publisher filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("fpc", 1);
	ros::Publisher compressed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cpc", 1);
	
	ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("obstacle_grid", 1);

	ros::Publisher sample_point_pub = nh.advertise<sensor_msgs::PointCloud2>("sp", 1);
	ros::Publisher graph_edge_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_edges", 1);
	ros::Publisher bad_graph_edge_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("bad_graph_edges", 1);
   	
   	ros::Publisher path_pub = nh.advertise<sensor_msgs::PointCloud2>("pathway", 1);

   	int time_to_pub = 100;
	ros::Rate count_rate(2); 
	int count = 0;

   	while(ros::ok() && (count < time_to_pub))
   	{
		g_shrunk_cloud.header.frame_id = "lidar_link";
		input_point_cloud_pub.publish(g_shrunk_cloud);

		g_filtered_cloud.header.frame_id = "lidar_link";
		filtered_cloud_pub.publish(g_filtered_cloud);

		compressed_cloud_pub.publish(cloudCompressor.getCloud());

		grid_pub.publish(cloudCompressor.getGrid());

		sample_points.header.frame_id = "rot";
		sample_point_pub.publish(sample_points);
			
		graph_edge_clouds.header.frame_id = "rot";
		graph_edge_clouds_pub.publish(graph_edge_clouds);

		pathway.pathCloud.header.frame_id = "rot";
		path_pub.publish(pathway.pathCloud);

		ros::spinOnce();
		count_rate.sleep();
		count++;
	}
    return 0;
}