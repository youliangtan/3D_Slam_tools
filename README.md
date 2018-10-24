# 3D_Slam_tools
Tools to work along side with LOAM 3D lidar slam and Octomaping. The main goal here is to generate a 2D occupancy map from a 3D Map. Essentially, after generating a 3D pcd map from LOAM, we just need to specify the required height range to generate a 2D map from 3D. 

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

## Record ROS bag file
Bag file is recorded to run the SLAM remotely after the recording process. Here Velodyne and IMU are used in the recording process of an indoor environment.

#### 1) Sensor Setup for Velodyne
Follow the [velodyne setup tutorial](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) and run the .launch file for pointcloud visualization on rviz. 


#### 2) Sensor Setup for VectorNav IMU
If IMU is used (Vectornav 100), use the ROS package [imu_vn_100](https://github.com/KumarRobotics/imu_vn_100). Run the .launch file below to receive the imu data on ROS topic `/imu/imu` and `/imu/rpy`

```
sudo chmod 666 /dev/ttyUSB0
roslaunch 3D_Slam_tools imu_vn_100.launch
````

After setup, conduct data collection via rosbag with a command

```
rosbag record -a
````

When ctrl-c, .bag file will be saved in current working directory


#### 3) Encoder Odometry Input

TO BE UPDATED 

Get encoder odometry from turtlebot after 'bringup' the robot.

```
roslaunch turtlebot_bringup minimal.launch
rostopic echo /odom
````


## Run Tools

### 1) 3D SLAM with input velodyne ROS Msg
Here we will have the ROS bag file for a indoor environmenmt. Run with LOAM package with ROS bag file (default path is in launch file). User can change the rosbag path and playback setting in the .launch file.
```
roslaunch 3D_Slam_tools loam_project.launch
````
To save .pcd and .bt files on fly, Run:
```
rosrun 3D_Slam_tools pcd2octomap_node
````

**For tuning, edit config file at `config/param.yaml`.


### 2) PCD Straigtener
to straighten the output .pcd file of a 3D map by using teleop of turtlebot (for convenience sake). User need to open RVIz and `rosrun turtlesim turtle_teleop_key` teleop to slowly straighten the map, then ctrl-c it to get the output .bt octomap file.

```
rosrun 3D_Slam_tools pcd2octomap <input_pcd_file> <output_bt_file> --rotate
````

Or just use launch file:

```
roslaunch 3D_Slam_tools straightener.launch input_path:="/home/youliang/catkin_ws/input_PC.pcd" output_path:="/home/youliang/catkin_ws/output_octo.bt"
````

** Use arrow key to control the rotation of the map.


### 3) 3D Octomap to 2D Occupancy map
Convert .bt file to slices occupancy map .pgm image file. Run map_saver in another terminal to save the map. Specify the `PATH_TO_BT_FILE`, optional float value of `Z_MIN` and `Z_MAX`. Both z-value are respected to the velodyne's position.

```
roslaunch 3D_Slam_tools octomap_mapping.launch path:=PATH_TO_BT_FILE z_min:=Z_MIN  z_max:=Z_MAX
rosrun map_server map_saver
````

Convert input .pgm to transparent .png map. Then user can use image editting tool to edit the map imagery.

```
convert input.pgm  -fuzz 20% -transparent white output.png
````

### 4) Odometry Handler Node
If IMU is used, this node will get /imu sensor msg, /point_cloud message, then transform it in a meaningful way to the SLAM node. Currently using vn100 imu for testing.

If encoder odom is used, /odom will be subcribed and publish to /tf, in terms of ` "odom_init"->"camera_init"->"encoder_odom" `.

To run the individual node:
```
rosrun 3D_Slam_tools odom_handler
````


### 5) TO BE CONTINUE
