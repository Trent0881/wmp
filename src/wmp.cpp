// Wobbler Motion Planning node
// Created April 14 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

// Includes map fixture and sensor model classes
//#include <ispl/models.h>

// File IO for C++
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>

// Define shorthand names for PCL points and clouds
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Point cloud of sensor measurements to be used to make our sensor model of parameters
PointCloud g_point_cloud_data;

ros::NodeHandle * nh_ptr;

// Mega-hacky function
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
        	//ROS_INFO("Adding Point(%f, %f, %f) to global cloud", x, y, value);
			g_point_cloud_data.push_back(Point(x, y, value));
			getting_z = false;
        	getting_x = true;
        }
    }

    if(g_point_cloud_data.points.size() > 0)
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
		ROS_WARN("NO DATA FROM FILE!?!?!?");
	}

	ros::Publisher input_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ipc", 1);
	
   	int time_to_wait = 10000; // ms
	ros::Rate count_rate(100); // ms
	int count = 0;

   	while(ros::ok() && (count < time_to_wait))
   	{
		g_point_cloud_data.header.frame_id = "lidar_link";
		input_point_cloud_pub.publish(g_point_cloud_data);
		
		ROS_INFO("Pubbed cloud with maybe points in it!");
			
		ros::spinOnce();
		count_rate.sleep();
		count++;
	}

    bool test_active = true;
    bool test_passed = true;

    //ros::Subscriber pc_sub = nh.subscribe("pc", 1, cloudCB);

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