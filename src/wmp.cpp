// Wobbler Motion Planning node
// Created April 14 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

#include <wmp/common.h>
#include <wmp/point_filter.h>
#include <wmp/cloud_compressor.h>
#include <wmp/grid.h>


// File IO for C++
#include <iostream>
#include <fstream>

// Point cloud of sensor measurements to be used to make our sensor model of parameters
PointCloud g_point_cloud_data;
PointCloud g_shrunk_cloud;
PointCloud g_filtered_cloud;

ros::NodeHandle * nh_ptr;

// DESCRIPTION
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

	CloudCompressor cloudCompressor;
	
	if(!cloudCompressor.setCloud(g_filtered_cloud))
	{
		ROS_WARN("Failed to set cloud data!");
	}

	if(!cloudCompressor.compress())
	{
		ROS_WARN("FAILED to compress cloud data!");
	}


	ros::Publisher input_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ipc", 1);
	ros::Publisher filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("fpc", 1);
	ros::Publisher compressed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cpc", 1);
	
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
		
		ROS_INFO("Published the same clouds again.");
			
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