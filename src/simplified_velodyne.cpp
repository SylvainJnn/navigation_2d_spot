#include "ros/ros.h"
#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

#define  PCL_EXPORTS
// afther 
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h> // for concatenateFields


sensor_msgs::PointCloud2 cloud_out;

//typedef sensor_msgs::PointCloud2 point;
typedef unsigned int uint16;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert message PointCloud2 to Velodyne PCL point cloud
    pcl::PCLPointCloud2 pcl_pointcloud2;                                // PCL Point cloud object 
    pcl_conversions::toPCL(*cloud_msg, pcl_pointcloud2);                // Convert from PointCloud2 (the msg gotten from the sensor) to PCL point cloud object  
    pcl::PointCloud<velodyne_pcl::PointXYZIRT>::Ptr pcl_cloud_ptr(new pcl::PointCloud<velodyne_pcl::PointXYZIRT>); // Velodyne PCL point cloud object
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_cloud_ptr);           // Convert PCL point cloud object to Velodyne PCL type 

    // Create PointXYZ point cloud and store data from Velodyne PointXYZIRT
    PointCloud::Ptr pcl_pointcLoud (new PointCloud);
    pcl_pointcLoud->header = pcl_cloud_ptr->header;

    for(int i=0; i< pcl_cloud_ptr->points.size(); i++) {
        pcl_pointcLoud->points.push_back (pcl::PointXYZ(pcl_cloud_ptr->points[i].x, pcl_cloud_ptr->points[i].y, pcl_cloud_ptr->points[i].z));
    }

    pcl_pointcLoud->height = pcl_cloud_ptr->height;
    pcl_pointcLoud->width = pcl_pointcLoud->points.size();
    pcl_pointcLoud->is_dense= pcl_cloud_ptr->is_dense;

    // Convert from PCL to sensor_msgs point cloud
    pcl::toROSMsg(*pcl_pointcLoud.get(),cloud_out );
}

int main (int argc, char** argv)
{

    ros::init (argc, argv, "simple_velodyne_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 10, cloud_cb);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_xyz", 10);


    ros::Rate loop_rate(10);
	int count = 0;
	ros::spinOnce();

    while (ros::ok()) {
        pub.publish(cloud_out);
        ros::spinOnce();
		loop_rate.sleep();
    }


}
