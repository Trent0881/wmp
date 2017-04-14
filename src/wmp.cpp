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
PointCloud g_shrunk_cloud;
PointCloud g_filtered_cloud;

ros::NodeHandle * nh_ptr;

class PointFilter
{
public:
	PointFilter();
	bool setPoint(Point new_point);
	bool translateAndShrink();
	bool cropPoint();
	Point getPoint(); 

private:

	float x_min;
	float x_max;
	float y_min;
	float y_max;
	float z_min;
	float z_max;

	Point point;
	Point center_point;
	bool first_time;
	float scale_factor;
};

PointFilter::PointFilter()
{
	first_time = true;
	scale_factor = 0.08;
	
	x_min = -0.25;
	x_max = 0.25;
	y_min = -0.25;
	y_max = 0.25;
	z_min = -0.3;
	z_max = 0.12;
}

bool PointFilter::setPoint(Point new_point)
{
	point = new_point;
	if(first_time == true)
	{
		first_time = false;
		// Transform the future points around this point
		center_point = point;
	}
}

bool PointFilter::translateAndShrink()
{
	// Translate to center the point around the center point, and shrink/expand by some constant factor for ease
	Point placeholder_point(point.x - center_point.x, point.y - center_point.y, point.z - center_point.z);
	point.x = (placeholder_point.x)*scale_factor;
	point.y = (placeholder_point.y)*scale_factor;
	point.z = (placeholder_point.z)*scale_factor;

	return true;
}

bool PointFilter::cropPoint()
{
	if(point.x > x_max || point.x < x_min ||
		point.y > y_max || point.y < y_min ||
		point.z > z_max || point.z < z_min)
	{
		return false;
	}
	return true;
}

Point PointFilter::getPoint()
{
	return point;
}

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

    PointFilter point_filter;
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
        	point_filter.setPoint(Point(x, y, value));

            if(point_filter.translateAndShrink())
        	{
        		g_shrunk_cloud.push_back(point_filter.getPoint());
			    if(point_filter.cropPoint())
	        	{
					g_filtered_cloud.push_back(point_filter.getPoint());
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

	ros::Publisher input_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("ipc", 1);
	ros::Publisher filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("fpc", 1);
	
   	int time_to_pub = 10000; // ms
	ros::Rate count_rate(500); // ms
	int count = 0;

   	while(ros::ok() && (count < time_to_pub))
   	{
		g_shrunk_cloud.header.frame_id = "lidar_link";
		input_point_cloud_pub.publish(g_shrunk_cloud);

		g_filtered_cloud.header.frame_id = "lidar_link";
		filtered_cloud_pub.publish(g_filtered_cloud);

		
		ROS_INFO("Pubbed a cloud or two!");
			
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