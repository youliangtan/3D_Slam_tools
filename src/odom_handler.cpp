/*   
  ODOM HANDLER NODE:  Handle Pointcloud odometry prior to LOAM 3D Slam process
  Odometry source: IMU and Encoder
  The node here also manage the tf transformation and publishing
*/


// select imu msg input by input mode 0 or 1

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// sensor msg
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



// using std::placeholders::_1;

// Sub: to /imu sensor_msgs/Imu
// Pub: to tf of imu/imu orientation
void imu_msgCallBack2(const sensor_msgs::Imu::ConstPtr& imuIn){

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
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "imu"));

}



// // =============================== Main IMU and PCD Transform Class ===========================================

// transfrom IMU according to imu/rpy topic
class OdomHandler
{
  private:

    ros::Publisher _pub_IMUfiltered;      ///< (high frequency) filtered orientation without trans imu publisher
    ros::Publisher _pub_IMUstatic;        ///< Juz static imu publisher
    ros::Publisher _pub_cloudTransformed; ///< (low frequency) filtered transformed cloud publisher
    ros::Subscriber _sub_IMUrpy;          ///< (high frequency) imu vector3 subscriber
    ros::Subscriber _sub_IMUmsg;          ///< (high frequency) imu msg subscriber
    ros::Subscriber _sub_encoderOdom;     ///< (high frequency) robot encoder msg subscriber
    ros::Subscriber _sub_velodyneCloud;   ///< (low frequency) velodyne raw cloud subscriber
    
    geometry_msgs::Vector3Stamped imu_rpy;
    sensor_msgs::Imu imu_msg;             // to get imu_msg, and change its orientation
    int is_encoderOdom_init;              // 1 if first odom value
    tf::Transform encoderOdom_init_tf;   // encoder transform odom value for first reading
    tf::TransformListener listener;



  public:

    //setup pub and sub
    bool setup(ros::NodeHandle& node)
    {
      // --- IMU filtered msg with standard imu format with rpy orientation
      _pub_IMUfiltered = node.advertise<sensor_msgs::Imu> ("/imu_filtered", 5);
      _pub_IMUstatic = node.advertise<sensor_msgs::Imu> ("/imu_static", 5);   // juz static
      
      // --- Publish static orientation pointcloud with transformed input pcd based of IMUrpy.orientation
      _pub_cloudTransformed =  node.advertise<sensor_msgs::PointCloud2> ("/velo_pointsTransformed", 1);

      // --- Source cloud from velodyne
      _sub_velodyneCloud = node.subscribe<sensor_msgs::PointCloud2::Ptr>("/velodyne_points", 2, &OdomHandler::cloudTransform_callback, this);
      
      // --- IMU rpy and rpy standard msg from IMU sensor
      _sub_IMUrpy = node.subscribe<geometry_msgs::Vector3Stamped>("/imu/rpy", 10, &OdomHandler::imu_rpyCallback, this);
      _sub_IMUmsg = node.subscribe<sensor_msgs::Imu>("/imu/imu", 10, &OdomHandler::imu_msgCallback, this);
      
      // --- Encoder odom msg from robot
      _sub_encoderOdom = node.subscribe<nav_msgs::Odometry::Ptr>("/odom", 10, &OdomHandler::encoderOdom_Callback, this);

      is_encoderOdom_init = 1;

      ROS_INFO("Done with Setup of Sub and Pub for node");
      return true;
    }



    // sub to /imu/imu to get retrieve all info of sensor
    void imu_msgCallback(const sensor_msgs::Imu::ConstPtr& imuIn){
      imu_msg = *imuIn;
    }



    // sub to /imu/ geometry_msgs/Vector3Stamped
    void imu_rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imuIn){

      // ----------------------- IMU filtered --------------

      // // Managing TF publisher
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(0, 0, 0.0) );
      imu_rpy = *imuIn;
      
      // set value tf publisher
      tf::Quaternion q;
      q.setRPY(imu_rpy.vector.x, -imu_rpy.vector.y, -imu_rpy.vector.z); //rpy to quaternion
      transform.setRotation(q);
      // br.sendTransform(tf::StampedTransform(transform, imu_rpy.header.stamp, "odom", "base_link")); //use imu orientation as odom

      // // Managing IMU Publisher
      sensor_msgs::Imu imuOut;  // set value to new imu publisher
      
      imuOut.header = imu_rpy.header;

      imuOut.orientation.x = q.x();
      imuOut.orientation.y = q.y();
      imuOut.orientation.z = q.z();
      imuOut.orientation.w = q.w();

      imuOut.angular_velocity = imu_msg.angular_velocity;
      imuOut.linear_acceleration = imu_msg.linear_acceleration;
      
      ROS_INFO("publishing imu_filtered out");

      _pub_IMUfiltered.publish(imuOut);

      // ------------------------ IMU Static ----------------

      imuOut.orientation.x = 0;
      imuOut.orientation.y = 0;
      imuOut.orientation.z = 0;
      imuOut.orientation.w = 0;
      imuOut.orientation_covariance[0] = -1;

      imuOut.angular_velocity.x = 0;
      imuOut.angular_velocity.y = 0;
      imuOut.angular_velocity.z = imuOut.angular_velocity.z;
      // imuOut.angular_velocity_covariance[0] = -1;

      imuOut.linear_acceleration.x = 0;
      imuOut.linear_acceleration.y = 0;
      imuOut.linear_acceleration.z = -9.8;

      _pub_IMUstatic.publish(imuOut);
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
  
    

    // Publish encoder odom msg to TF (camera_init to encoderOdom)
    void encoderOdom_Callback(const nav_msgs::Odometry::Ptr _encoderOdom){

      // encoder init till get tf of laser_odom
      if (is_encoderOdom_init == 1){
        // // Managing TF publisher

        float x, y, z, w;
        x =  _encoderOdom->pose.pose.position.x;
        y =  _encoderOdom->pose.pose.position.y;
        z =  _encoderOdom->pose.pose.position.z;
        tf::Vector3 vec(x,y,z);

        // quaternion to rpy
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(_encoderOdom->pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        //rpy to quaternion
        tf::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        encoderOdom_init_tf.setOrigin(vec);
        encoderOdom_init_tf.setRotation(quat);

        tf::StampedTransform laser_transform;

        try{
          listener.lookupTransform("/camera_init", "/laser_odom",  ros::Time(0), laser_transform);
          // once get the tf from laser_transform, change is_encoderOdom_init value to escape id statement
          is_encoderOdom_init = 0;   
        }
        catch(tf::TransformException ex){      
          ROS_INFO("Waiting for laser_odom tf msg...");    
        }
        
      }

      // // Managing TF publisher
      static tf::TransformBroadcaster br;
      tf::Transform transform;

      float x, y, z, w;
      x = _encoderOdom->pose.pose.position.x;
      y = _encoderOdom->pose.pose.position.y;
      z = _encoderOdom->pose.pose.position.z;
      tf::Vector3 vec(x,y,z);

      x = _encoderOdom->pose.pose.orientation.x;
      y = _encoderOdom->pose.pose.orientation.y;
      z = _encoderOdom->pose.pose.orientation.z;
      w = _encoderOdom->pose.pose.orientation.w;
      tf::Quaternion quat(x,y,z,w);
      
      transform.setOrigin(vec);
      transform.setRotation(quat);

      br.sendTransform(tf::StampedTransform(encoderOdom_init_tf, imu_rpy.header.stamp, "odom_init", "camera_init")); //use imu orientation as odom
      br.sendTransform(tf::StampedTransform(transform, imu_rpy.header.stamp, "odom_init", "encoder_odom")); //use imu orientation as odom

    }

};

// // ==================================== End Class ===========================================





// // ======================================= Main ================================================

int main(int argc, char** argv){

  ros::init(argc, argv, "odom_handler_node");
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

    OdomHandler odom_handler;
    if (odom_handler.setup(node)) {
      // successful initialization 
      ros::spin();
    }   
  }
  
  return 0;
};
