/*
 *  Created By: Tan You Liang, Sept 2018
 *  - Node to capture Pointcloud topic to Octomap .bt format
 *  - Created for Testing
*/

#include <iostream>
#include <assert.h>
#include <signal.h>
#include <string>

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
// #include <pcl/conversions.h>
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

// interrupt handler
void save_interrupt(int s){
    // octo tree cooefficient
    octomap::OcTree tree( 0.05 );
 
    for (int i = 0; i < output_cloud->points.size() ; i++)
    {
        tree.updateNode( octomap::point3d(  output_cloud->points[i].x, 
                                            output_cloud->points[i].y, 
                                            output_cloud->points[i].z), true );
    }
    // update octomap
    tree.updateInnerOccupancy();

    // output file to current working dir
    stringstream ss; //convert int to str
    ss << output_count;
    string idx = ss.str();
    tree.writeBinary( "output_bt" + idx + ".bt" );    // saving .bt file
    pcl::io::savePCDFileASCII ("output_pcd" + idx + ".pcd", *output_cloud);

    cout<<"SUCCESS: Output .bt file is saved!! "<<endl;
    output_count++;
 
}


int main( int argc, char** argv )
{
    char c;
    // start ros
    ros::init(argc, argv, "pcd2octo");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2, octo_callback);
    ros::Rate r(100);
    
    // create > save .bt file interrupt
    signal(SIGINT,save_interrupt);

    ros::spin();
    
    return 0;

}