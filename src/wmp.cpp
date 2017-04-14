// Wobbler Motion Planning node
// Created April 14 2017 by Trent Ziemer
// Last updated XXX by Trent Ziemer

// Includes map fixture and sensor model classes
//#include <ispl/models.h>

// File IO for C++
#include <iostream>
#include <fstream>

#include <ros/ros.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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

    float value;
    float value2;
    int j = 1;
    if(0) // THIS IS A HACK DONT EVEN BOTHER TRYING TO COMPREHEND
    {
    	while(input_data >> value)
    	{
    		value2 =  1.732051*j/(88*sqrt(3));
    		for(int i = 0; i < value; i++)
    		{
    			g_point_cloud_data.push_back(Point(value2, value2, value2));
    		}
    		// ROS_INFO("Added %f of %f", value, value2*sqrt(3));
    		j++;
    	}
    }
    else // This is real...a real state machine
    {
    bool getting_x = true;
    bool getting_y = false;
    bool getting_z = false;

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

	if(!nh_ptr->getParam("filename", filename))
	{
		ROS_WARN("Failed to get filename for input data source!");
	}


    bool test_active = true;
    bool test_passed = true;

    //ros::Subscriber pc_sub = nh.subscribe("/ispl/point_cloud", 1, cloudCB);
    
    // Publish point cloud for reference by other nodes, not currently really useful
	//ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/meas_pc", 1);
	//ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2> ("/ispl/map_pc", 1);

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