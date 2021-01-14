#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "elas.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include<tf/transform_listener.h>
#include<geometry_msgs/PointStamped.h>


cv::Mat leftImg;
cv::Mat rightImg;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void left(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("left height = %d, width = %d",msg->height, msg->width);
	cv_bridge::CvImagePtr c_ptr;
	c_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
	leftImg = c_ptr->image;
	cv::imwrite("/home/sagacious/stereo_ws/l.png",leftImg);
}

void right(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("right height = %d, width = %d",msg->height, msg->width);
	cv_bridge::CvImagePtr c_ptr;
	c_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
	rightImg = c_ptr->image;
	cv::imwrite("/home/sagacious/stereo_ws/r.png",rightImg);
}

void camerapose(const nav_msgs::Odometry::ConstPtr& msg)
{
	static tf::TransformBroadcaster broadcaster;
	// std::cout<<msg->pose<<std::endl;
	double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
	double ang_x = msg->pose.pose.orientation.x;
	double ang_y = msg->pose.pose.orientation.y;
	double ang_z = msg->pose.pose.orientation.z;
	double ang_w = msg->pose.pose.orientation.w;
	std::cout<<x<<" "<<y<<" "<<z<<" "<<ang_x<<" "<<ang_y<<" "<<ang_z<<" "<<ang_w<<std::endl;

  	tf::TransformBroadcaster br;
  	tf::Transform transform;

	transform.setOrigin( tf::Vector3(x, y, z) );
    transform.setRotation( tf::Quaternion(ang_x, ang_y, ang_z, ang_w) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "stereo_link"));



}





int main(int argc, char *argv[])
{

	ros::init(argc, argv, "stereo_ros");
	// TestSystemNode TestSystem;
	ros::NodeHandle nh;
	ros::Subscriber cam_left = nh.subscribe("/stereocamera/left/image_raw",1,left);
	ros::Subscriber cam_right = nh.subscribe("/stereocamera/right/image_raw",1,right);

	ros::Subscriber pose_ = nh.subscribe("/ground_truth/camera_state",1,camerapose);
	ros::Publisher pointPublisher = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 2);
	ros::Publisher depthPublisher = nh.advertise<sensor_msgs::Image>("depth", 1);
	tf::TransformListener listener(ros::Duration(10));

	StereoEfficientLargeScale* stereo_ = new StereoEfficientLargeScale(0, 128);
	const double camera_factor = 1000;
	const double camera_cx = 424;
	const double camera_cy = 240;
	const double camera_fx = 518.0;
	const double camera_fy = 519.0;

	sensor_msgs::ImagePtr d_msg;
	PointCloud::Ptr cloud ( new PointCloud );


  	tf::TransformBroadcaster br;
  	tf::Transform transform;

	transform.setOrigin( tf::Vector3(0, 0, 0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "stereo_link"));


    // ros::spin();
	while(ros::ok())
	{
		ros::spinOnce();

		if(leftImg.data == NULL || rightImg.data == NULL)
		{
        	continue;
    	}

    	double t = (double)cvGetTickCount();

    	cv::Mat gradx = cv::Mat::zeros(leftImg.rows, leftImg.cols, CV_32F);
    	cv::Mat grady = cv::Mat::zeros(leftImg.rows, leftImg.cols, CV_32F);
    	cv::Mat mag =  cv::Mat::zeros(leftImg.rows, leftImg.cols, CV_32F);
    	cv::Mat img;
    	cv::GaussianBlur( leftImg, img, cv::Size( 3, 3 ), 0, 0 );
    	cv::Scharr(img, gradx, CV_32F, 1, 0, 1/32.0);
    	cv::Scharr(img, grady, CV_32F, 0, 1, 1/32.0);
    	cv::magnitude(gradx,grady,mag);
    	cv::Mat ipOut;
    	cv::threshold(mag, ipOut, 15, 0, THRESH_TOZERO);
    	cv::Mat dest;
    	stereo_->operator ()(leftImg,rightImg,dest,100,mag);

    	t=(double)cvGetTickCount()-t;
    	printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));

    	cv::Mat map = stereo_->GetDenseMap();
    	cv::Mat depth;
    	map.convertTo(depth, CV_16UC1);
		d_msg = cv_bridge::CvImage(std_msgs::Header(),"mono16",depth*50).toImageMsg();
		depthPublisher.publish(d_msg);



		// 遍历深度图
		for (int m = 0; m < depth.rows; m++)
			for (int n=0; n < depth.cols; n=n+10)
			{
				// 获取深度图中(m,n)处的值
				ushort d = depth.ptr<ushort>(m)[n];
				// d 可能没有值，若如此，跳过此点
				if (d == 0)
					continue;
				// d 存在值，则向点云增加一个点
				PointT p;

				// 计算这个点的空间坐标
				double c = double(d) / camera_factor;
				double a = (n - camera_cx) * c / camera_fx;
				double b = (m - camera_cy) * c / camera_fy;

				// p.z = double(d) / camera_factor;
				// p.x = (n - camera_cx) * p.z / camera_fx;
				// p.y = (m - camera_cy) * p.z / camera_fy;
				p.z = -0.866025*b+0.5*c;
				p.y = a;
				p.x = 0.5*b+0.866025*c;


				// 从rgb图像中获取它的颜色
				// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
				p.r = leftImg.ptr<uchar>(m)[n*3];
				p.g = leftImg.ptr<uchar>(m)[n*3+1];
				p.b = leftImg.ptr<uchar>(m)[n*3+2];


				geometry_msgs::PointStamped laser_point;  //这里定义了一个相对于laser_link坐标系的点
				laser_point.header.frame_id="stereo_link";
				laser_point.header.stamp=ros::Time();
				laser_point.point.x=p.x;     //设置相对于laser_link坐标系的坐标
				laser_point.point.y=p.y;
				laser_point.point.z=p.z;

				geometry_msgs::PointStamped base_point;
				listener.transformPoint("base_footprint", laser_point, base_point); //Transform a Stamped Point Message into the target frame

				p.x = base_point.point.x;
				p.y = base_point.point.y;
				p.z = base_point.point.z;

				// 把p加入到点云中
				cloud->points.push_back( p );


			}
		// 设置并保存点云
		cloud->height = 1;
		cloud->width = cloud->points.size();
		std::cout<<"point cloud size = "<<cloud->points.size()<<std::endl;
		cloud->is_dense = false;
		sensor_msgs::PointCloud2 output;
				// //把点云转化为ros消息
		pcl::toROSMsg(*cloud, output);

		output.header.frame_id = "base_footprint";
		pointPublisher.publish(output);
	}


	return 0;
}