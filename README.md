# 3D_Slam_tools
Tools to work along side with LOAM 3D lidar slam and Octomaping

## Setup Environment
- ROS
- Download [LOAM Velodyne Mapping](https://github.com/yutingkevinlai/velodyne_slam), with dynamic object removal
- [octo_mapping](https://github.com/OctoMap/octomap_mapping) (git clone)
- map_saver  (apt-get install navigation)

### Compilation
```
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
````

## Run Tools

### 3D SLAM
Run with LOAM package with ROS bag file (default path is in launch file)
```
roslaunch 3D_Slam_tools loam_project.launch
````
To save .pcd and .bt files on fly, Run:
```
rosrun 3D_Slam_tools pcd2octomap_node
````
Convert .bt file to slices ocupancy map png
```
roslaunch 3D_Slam_tools octomap_mapping.launch path:=PATH_TO_BT_FILE z_min:=OPTIONAL_FLOAT  z_max:=OPTIONAL_FLOAT
rosrun map_server map_saver
````
edit config file in `config/param.yaml` folder


### Straigtener
to straighten the output .pcd file of a 3D map by using teleop of turtlebot (for convenience sake). User need to open RVIz and `rosrun turtlesim turtle_teleop_key` teleop to slowly straighten the map, then ctrl-c it to get the output .bt octomap file.

```
rosrun 3D_Slam_tools pcd2octomap <input_pcd_file> <output_bt_file> --rotate
````

Or just use launch file:

```
roslaunch 3D_Slam_tools straightener.launch input_path:="/home/youliang/catkin_ws/input_PC.pcd" output_path:="/home/youliang/catkin_ws/output_octo.bt"
````

** Use arrow key to control the rotation of the map.


### IMU TF publisher
This node will get /imu sensor msg, /point_cloud message, then transform it in a meaningful way to the SLAM node. Currently using vn100 imu for testing.

Use ROS driver below to read imu publish data, in /imu/imu and /imu/imu topics

```
git clone git@github.com:KumarRobotics/imu_vn_100.git
````

To run the node:
```
rosrun 3D_Slam_tools imu_TFpublisher
````

### Map your own bag file

#### Velodyne
Follow the [velodyne setup tutorial](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) and run the .launch file for pointcloud visualization on rviz. 


### VectorNav IMU
If IMU is used (Vectornav 100), use the ROS package [imu_vn_100](https://github.com/KumarRobotics/imu_vn_100). Run the .launch file to receive the imu data on ROS topic `/imu/imu` and `/imu/rpy`

After setup, conduct data collection via rosbag with a command

```
rosbag record -a
````

When ctrl-c, .bag file will be saved in current working directory



### TO BE CONTINUE
