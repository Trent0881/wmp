// Wobbler Motion Planning node
// Created April 14 2017 by Trent Ziemer
// Last updated May 3 2017 by Trent Ziemer

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
Point g_center_point;

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
        	g_center_point = Point(x, y, value);
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
	int number_of_points_per_line = 30; //std::max((double)50, 100*sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
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

	int cells_per_side = 30;
	FreeSpaceGraph planningGraph(&grid, cells_per_side, cells_per_side);
	
	PointCloud sample_points = planningGraph.getPointCloud();

	ROS_INFO("Connecting nodes on graph");
	float connection_radius = (float)10/cells_per_side;
	
	planningGraph.connectNodes(connection_radius);

	ROS_INFO("Building node-based pathway planner");
	PathSearcher pathway(planningGraph.getNodes(), Point(0,0,0), Point(-2, -2, 0));

	ROS_INFO("Creating graph edge clouds");

	PointCloud graph_edge_clouds;
	for(int i = 0; i < planningGraph.nodeList.size(); i++)
	{
		//ROS_INFO("Node %d [%f, %f] at (%f, %f) is connected to:", i, planningGraph.nodeList[i].x, planningGraph.nodeList[i].y, planningGraph.nodeList[i].point.x, planningGraph.nodeList[i].point.y);
		for(int j = 0; j < planningGraph.nodeList[i].nearbyNodes.size(); j++)
		{
			//ROS_INFO("--- #%d node %d at (%f, %f)", j, planningGraph.nodeList[i].nearbyNodes[j].distantNode->id, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.x, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.y);
			
			PointCloud edge_cloud = generateCloudLine(planningGraph.nodeList[i].point.x, planningGraph.nodeList[i].point.y, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.x, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.y);
			
			for(int k = 0; k < edge_cloud.size(); k++)
			{
				graph_edge_clouds.push_back(edge_cloud[k]);
			}
		}
	}

	// Linear collisions
	PointCloud bad_graph_edge_clouds;
	for(int i = 0; i < planningGraph.nodeList.size(); i++)
	{
		//ROS_INFO("Node %d at (%f, %f) is connected to:", i, planningGraph.nodeList[i].point.x, planningGraph.nodeList[i].point.y);
		for(int j = 0; j < planningGraph.nodeList[i].nearbyNodes.size(); j++)
		{
			//ROS_INFO("--- #%d node %d at (%f, %f)", j, planningGraph.nodeList[i].nearbyNodes[j].distantNode->id, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.x, planningGraph.nodeList[i].nearbyNodes[j].distantNode->point.y);
			
			//PointCloud bad_edge_cloud = generateCloudLine(g_bad_nodes[i].x, g_bad_nodes[i].y, g_bad_nodes_two[i].x, g_bad_nodes_two[i].y);
			
			//for(int k = 0; k < bad_edge_cloud.size(); k++)
			{
				//bad_graph_edge_clouds.push_back(bad_edge_cloud[k]);
			}
		}
	}

	ROS_INFO("Done with processing and planning!");

	ros::Publisher input_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ipc", 1);
	ros::Publisher filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("fpc", 1);
	ros::Publisher compressed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cpc", 1);
	
	ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("obstacle_grid", 1);

	ros::Publisher sample_point_pub = nh.advertise<sensor_msgs::PointCloud2>("sp", 1);
	ros::Publisher graph_edge_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("graph_edges", 1);
	ros::Publisher bad_graph_edge_clouds_pub = nh.advertise<sensor_msgs::PointCloud2>("bad_graph_edges", 1);
   	
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
		
		//bad_graph_edge_clouds.header.frame_id = "rot";
		//bad_graph_edge_clouds_pub.publish(bad_graph_edge_clouds);

		ros::spinOnce();
		count_rate.sleep();
		count++;
	}

    bool test_active = true;
    bool test_passed = true;

    if(test_active == true)
    {

    }

    // Check this-node functional testing and report to user
    if(test_passed == false)
    {
    	ROS_WARN("FUNCTIONAL TESTING FAILED: Something is borked");
    }
    else
    {
    	nh_ptr->setParam("/success", true);
    }

    // Tell test node that we are done, so it can start doing things
    nh_ptr->setParam("/planning_done", true);

    return 0;
}