#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include "point_cloud_utilities/pcl_utilities.hpp"

using namespace std;

ros::Publisher Publish_Cloud;

void publishPointCloudFromPCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	for(int i=0;i<cloud->points.size();i++)
		pcl_cloud.points.push_back(cloud->points[i]);
	pcl_cloud.width = cloud->width;
	pcl_cloud.height = cloud->height;
	pcl_cloud.is_dense = true;
	pcl_cloud.header.frame_id = "pcl";
	PCLUtilities::publishPointCloud<pcl::PointXYZRGB>(pcl_cloud,Publish_Cloud);   
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publish_cloud_from_pcd");
	ros::NodeHandle pnh("~");
	Publish_Cloud = pnh.advertise<sensor_msgs::PointCloud2> ("/approximate_bb/cloud", 1);
 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 	std::string filename;
    pnh.param<std::string>("filename", filename, "filename");
 	pcl::io::loadPCDFile(filename,*cloud);
 	ros::Rate r(1);
	while(ros::ok())
	{
		publishPointCloudFromPCD(cloud);
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}

