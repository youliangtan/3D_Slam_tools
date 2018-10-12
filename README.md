# 3D_Slam_tools
Tools to work along side with LOAM 3D lidar slam and Octomaping

## Setup Environment
- ROS
- Download LOAM Velodyne Mapping, 
`git clone git@github.com:yutingkevinlai/velodyne_slam.git` with dynamic object removal

- Octomapping
- map_saver



## Run Tools

### Straigtener
to straighten the output .pcd file of a 3D map by using teleop of turtlebot (for convenience sake). User need to open RVIz and teleop to slowly straighten the map, then ctrl-c it to get the output .bt octomap file.

### IMU TF publisher
This node will get /imu sensor msg, /point_cloud message, then transform it in a meaningful way to the SLAM node. Currently using vn100 imu for testing.

Use ROS driver below to read imu publish data, in /imu/imu and /imu/imu topics

> git clone git@github.com:KumarRobotics/imu_vn_100.git


### TO BE CONTINUE
