// select imu msg input by input mode 0 or 1

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// sensor msg
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



// using std::placeholders::_1;

// Sub: to /imu sensor_msgs/Imu
// Pub: to tf of imu/imu orientation
void imu_msgCallBack2(const sensor_msgs::Imu::ConstPtr& imuIn){
  ROS_INFO("test exist %f", imuIn->linear_acceleration.z);   // assume accel input is in terms from 0-1

  // quaternion to rpy
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  // tf::Vector3 acc; //vector 3 is avail on pcl
  // acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  // acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  // acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  // tf
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0.0) );
  
  // set value tf publisher
  tf::Quaternion q(imuIn->orientation.w, imuIn->orientation.x, imuIn->orientation.y, imuIn->orientation.z);

  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));

}



// // =============================== Main IMU and PCD Transform Class ===========================================

// transfrom IMU according to imu/rpy topic
class IMU_CloudTransform
{
  private:

    ros::Publisher _pub_IMUfiltered;      ///< (high frequency) filtered orientation without trans imu publisher
    ros::Publisher _pub_cloudTransformed;    ///< (low frequency) filtered transformed cloud publisher
    ros::Subscriber _sub_IMUrpy;          ///< (high frequency) imu vector3 subscriber
    ros::Subscriber _sub_IMUmsg;          ///< (high frequency) imu msg subscriber

    ros::Subscriber _sub_velodyneCloud;   ///< (low frequency) velodyne raw cloud subscriber
    
    geometry_msgs::Vector3Stamped imu_rpy;
    sensor_msgs::Imu imu_msg;             // to get imu_msg, and change its orientation

  public:

    //setup pub and sub
    bool setup(ros::NodeHandle& node)
    {

      _pub_IMUfiltered = node.advertise<sensor_msgs::Imu> ("/imu_filtered", 5);
      _pub_cloudTransformed =  node.advertise<sensor_msgs::PointCloud2> ("/velo_pointsTransformed", 1);
      
      _sub_IMUrpy = node.subscribe<geometry_msgs::Vector3Stamped>("/imu/rpy", 10, &IMU_CloudTransform::imu_rpyCallback, this);
      _sub_velodyneCloud = node.subscribe<sensor_msgs::PointCloud2::Ptr>("/velodyne_points", 2, &IMU_CloudTransform::cloudTransform_callback, this);
      _sub_IMUmsg = node.subscribe<sensor_msgs::Imu>("/imu/imu", 10, &IMU_CloudTransform::imu_msgCallback, this);

      ROS_INFO("Done with Setup of Sub and Pub for node");
      return true;
    }

    // sub to /imu/imu to get retrieve all info of sensor
    void imu_msgCallback(const sensor_msgs::Imu::ConstPtr& imuIn){
      imu_msg = *imuIn;
    }


    // sub to /imu/ geometry_msgs/Vector3Stamped
    void imu_rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imuIn){

      // tf
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(0, 0, 0.0) );
      imu_rpy = *imuIn;
      
      // set value tf publisher
      tf::Quaternion q;
      q.setRPY(imu_rpy.vector.x, -imu_rpy.vector.y, -imu_rpy.vector.z); //rpy to quaternion
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, imu_rpy.header.stamp, "odom", "base_link")); //use imu orientation as odom

      // set value to new imu publisher
      sensor_msgs::Imu imuOut;
      // imuOut.header = "imu";      
      imuOut.header = imu_rpy.header;

      imuOut.orientation.x = q.x();
      imuOut.orientation.y = q.y();
      imuOut.orientation.z = q.z();
      imuOut.orientation.w = q.w();

      imuOut.angular_velocity = imu_msg.angular_velocity;
      imuOut.linear_acceleration = imu_msg.linear_acceleration;
      
      ROS_INFO("publishing imu_filtered out");

      _pub_IMUfiltered.publish(imuOut);
    }


    // transform pcd to imu rpy frame
    void cloudTransform_callback(const sensor_msgs::PointCloud2::Ptr _cloud){

      pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::fromROSMsg( *_cloud, *velo_cloud);

      Eigen::Affine3f transform = Eigen::Affine3f::Identity(); 

      // The same rotation matrix as before; theta radians around Z axis
      transform.rotate (Eigen::AngleAxisf (-imu_rpy.vector.x, Eigen::Vector3f::UnitX()));
      transform.rotate (Eigen::AngleAxisf (-imu_rpy.vector.y, Eigen::Vector3f::UnitY()));
      transform.rotate (Eigen::AngleAxisf (-imu_rpy.vector.z, Eigen::Vector3f::UnitZ()));

      // // Print the transformation
      // std::cout << transform.matrix() << std::endl;

      // // Executing the transformation
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*velo_cloud, *transformed_cloud, transform);

      // Output msg
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*transformed_cloud, msg);
      msg.header.stamp = _cloud->header.stamp;
      msg.header.frame_id = _cloud->header.frame_id;
      _pub_cloudTransformed.publish(msg);
    }
  


};

// // ==================================== End Class ===========================================





// // ======================================= Main ================================================

int main(int argc, char** argv){

  ros::init(argc, argv, "imu_tf_broadcaster");
  ros::NodeHandle node;

  // Get Arguments
  int mode = 0; // default
  if (argc > 1) {
    mode = std::atoi(argv[1]);
    std::cout << "Mode is: " << mode << std::endl;
  }

  // Mode 1 or 2
  if (mode == 1){  
    ROS_INFO("Using ms sensor_msgs/Imu!!!");
    ros::Subscriber sub = node.subscribe<sensor_msgs::Imu>("/imu/imu", 10, imu_msgCallBack2);
    ros::spin();
  }

  else{
    ROS_INFO("Using msg geometry_msgs/Vector3Stamped!!");

    IMU_CloudTransform imu_cloudtransform;
    if (imu_cloudtransform.setup(node)) {
      // successful initialization 
      ros::spin();
    }   
  }
  
  return 0;
};
