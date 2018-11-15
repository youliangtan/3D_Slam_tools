/*
    Created By: Tan You Liang, Nov 2018
    Simple as named, to save pcd topic to .pcd data
    USAGE:  rosrun 3D_Slam_tools pcd_topic_saver [pcd_topic]
            Ctrl-C and will save another copy, ctrl-Z to stop completely
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

#define USAGE "\n USAGE: rosrun 3D_Slam_tools pcd_topic_saver [/pcd_topic] \n" \
    "  .pcd will be saved in current dir \n"


pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
int output_count;

// save latest 3d point cloud to local
void pcd_callback(const sensor_msgs::PointCloud2ConstPtr& _cloud){
    pcl::fromROSMsg( *_cloud, *output_cloud);
    cout<<"point cloud loaded, point size = "<< output_cloud->points.size()<<endl;
}


// interrupt handler to save .pcd file
void save_interrupt(int s){

    // output file to current working dir
    stringstream ss; //convert int to str
    ss << output_count;
    string idx = ss.str();
    pcl::io::savePCDFileASCII ("output_pcd" + idx + ".pcd", *output_cloud);

    cout<<" SUCCESS: Output .pcd file is saved!! "<<endl;
    output_count++;
 
}


int main( int argc, char** argv )
{
    char c;
    // start ros
    ros::init(argc, argv, "pcd_topic_saver");
    
    if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
        ROS_ERROR("%s", USAGE);
        exit(-1);
    }

    std::string topic_name = argv[1]; // Name of the current exec program
    std::cout << " Sub topic is: " << topic_name << std::endl;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(topic_name, 2, pcd_callback);
    ros::Rate r(100);
    
    // create > save .bt file interrupt
    signal(SIGINT,save_interrupt);

    ros::spin();
    
    return 0;

}