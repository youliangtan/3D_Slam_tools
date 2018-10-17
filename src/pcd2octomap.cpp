/*
Usage: pcd2octomap <input_file> <output_file> --rotate
*/

#include <iostream>
#include <assert.h>
 
// ROS
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Twist.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
 
// octomap 
#include <octomap/octomap.h>

using namespace std;

double octotree_co =  0.05; //octo tree coefficient
float roll = 0;
float pitch = 0;
float angle_resolution = 0.002;


// convert pointcloudXYZ data to octotree format, and output it
void octotree_conversion(pcl::PointCloud<pcl::PointXYZ> cloud, string output_file){

    cout<<"copy data into octomap..."<<endl;
    // octo tree coefficient
    octomap::OcTree tree( octotree_co );
 
    for (int i = 0; i < cloud.points.size() ; i++)
    {
        tree.updateNode( octomap::point3d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z), true );
    }
 
    // update octomap
    tree.updateInnerOccupancy();
    // saving .bt file
    tree.writeBinary( output_file );
    cout<<"Done All!!"<<endl;
 
}


// leverage on turtlebot teleop arrow key control
void teleop_callback(const geometry_msgs::Twist::ConstPtr& KeyIn){

    roll = roll + angle_resolution*KeyIn->linear.x;
    pitch = pitch + angle_resolution*KeyIn->angular.z;

    // KeyIn->angular.z
    std::cout << "###Entered Key Arrow UP Down " <<  KeyIn->linear.x << std::endl;
    std::cout << "###Entered Key Arrow <----> " <<  KeyIn->angular.z << std::endl;

}


void getKey(){
    char c;
    if(read(0, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    std::cout << "##THIS IS KEY "<< c << std::endl;
}


// rotate via getchar
void pointcloud_rotate(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){

    Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 

    // getKey();

    // The same rotation matrix as before; theta radians around Z axis
    transform.rotate (Eigen::AngleAxisf (roll, Eigen::Vector3f::UnitX()));
    transform.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (1.5, Eigen::Vector3f::UnitZ()));

    // // Print the transformation
    // printf ("\n using an Affine3f\n");
    // std::cout << transform.matrix() << std::endl;

    // // Executing the transformation
    pcl::transformPointCloud (*input_cloud, *output_cloud, transform);

}





// // ======================================= Main ================================================

int main( int argc, char** argv )
{
    if (argc < 3 || pcl::console::find_switch (argc, argv, "-h") )
    {
        cout<<" - Run this script to convert .pcd file to .bt file"<<endl;
        cout<<"Usage: pcd2octomap <input_file> <output_file>"<<endl;
        cout<<"Usage: pcd2octomap <input_file> <output_file> -s <size>"<<endl;
        cout<<" - Rotate Mode>"<<endl;
        cout<<"Usage: pcd2octomap <input_file> <output_file> --rotate "<<endl;
        return -1;
    }

    //get arg point size
    if (pcl::console::find_switch (argc, argv, "-s")){
        int input_idx = pcl::console::find_argument (argc, argv, "-s") + 1;
        std::stringstream ss( argv[input_idx] );
        if ( !(ss >> octotree_co))
        std::cout << "Invalid double...\n";
    } 
    std::cout << "Input double arg for '-s' is " << octotree_co << std::endl;

    // io file
    string input_file = argv[1], output_file = argv[2];
    cout <<"Input File is: " << input_file << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPCDFile<pcl::PointXYZ> ( input_file, *cloud );
    
    cout<<"OctoTree coefficient is = "<< octotree_co <<endl;
    cout<<"point cloud loaded, point size = "<<cloud->points.size()<<endl;
 
    // Rotate?
    if (pcl::console::find_switch (argc, argv, "--rotate")){

        std::cout << "\t Rotate Mode!!!\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


        ros::init(argc, argv, "PCD_Rotate");
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/pcd", 100);
        ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel", 2, teleop_callback);
        ros::Rate loop_rate(5); //1hz

        while (ros::ok())
        {
            // rotate poincloud
            pointcloud_rotate( cloud, transformed_cloud);
            
            // Output msg
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*transformed_cloud, msg);
            msg.header.frame_id = "world";
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        octotree_conversion(*transformed_cloud, output_file);
    } 
    else{
        octotree_conversion(*cloud, output_file);
    }



    return 0;

}