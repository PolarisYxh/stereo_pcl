#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>



#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{

	ros::init(argc, argv, "sparse_ros");
	ros::NodeHandle n;
	// ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	ros::Publisher pointPublisher = n.advertise<sensor_msgs::PointCloud2>("sparse", 2);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	float a;

    if (pcl::io::loadPLYFile<pcl::PointXYZRGB>("/home/yxh/Documents/construct/src/sparse_ros/map.ply", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        system("PAUSE");
        return (-1);
    }

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{

        a = cloud->points[i].y;
		cloud->points[i].y = cloud->points[i].z;
		cloud->points[i].z = a;
		cloud->points[i].z -= 0.6;
		a = cloud->points[i].y;

		cloud->points[i].y = -cloud->points[i].x;
		cloud->points[i].x = a;
	}

	sensor_msgs::PointCloud2 output;
	// //把点云转化为ros消息
	pcl::toROSMsg(*cloud, output);
	output.header.frame_id = "base_footprint";
	while (ros::ok())
	{
		pointPublisher.publish(output);
		ros::Rate loop_rate(10);
	}



	return 0;
}