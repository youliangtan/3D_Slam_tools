#include <iostream>
#include <assert.h>
#include <signal.h>
#include <string>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//octomap 
#include <octomap/octomap.h>

using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
int output_count;

// save latest 3d point cloud to local
void octo_callback(const sensor_msgs::PointCloud2ConstPtr& _cloud){
    pcl::fromROSMsg( *_cloud, *output_cloud);
    cout<<"point cloud loaded, point size = "<< output_cloud->points.size()<<endl;
}


int main( int argc, char** argv )
{

    // start ros
    ros::init(argc, argv, "pc_frame_conversion");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/map_points", 1000);
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, octo_callback);

    ros::Rate r(100);

    // publisher
    while (ros::ok())
    {
        output_cloud->header.frame_id = "/map";                 //change frame HERE!!!!!!!
        sensor_msgs::PointCloud2 point_cloud;
        pcl::toROSMsg(*output_cloud, point_cloud);

        pub.publish(point_cloud);
        ros::spinOnce();
    }
    
    return 0;

}
