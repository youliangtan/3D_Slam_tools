# 3D_Slam_tools
Tools to work along side with LOAM 3D lidar slam and Octomaping. The main goal here is to generate 
a 2D occupancy map from a 3D Map.

Generated 3D Map and Sliced Map with LOAM and surrounding tools
![alt text](/documentation/compare-map.png?)

## Setup Environment
- ROS, obviously
- Clone [LOAM Velodyne Mapping](https://github.com/yutingkevinlai/velodyne_slam), with dynamic object removal
- [octo_mapping](https://github.com/OctoMap/octomap_mapping) (git clone)
- map_saver  (apt-get install navigation)

### Compilation

This package is tested on ROS Kinetic. For ROS Melodic, pls refer [here](##Troubleshooting)
```bash
cd catkin_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg loam_velodyne 3D_Slam_tools
```

## Record ROS bag file
Bag file is recorded to run the SLAM remotely after the recording process. Here Velodyne and IMU are used 
in the recording process of an indoor environment.

### 1) Sensor Setup for Velodyne
Follow the [velodyne setup tutorial](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16) 
and run the .launch file for pointcloud visualization on rviz. 

### 2) Sensor Setup for VectorNav IMU (Optional)
If IMU is used (Vectornav 100), use the ROS package [imu_vn_100](https://github.com/KumarRobotics/imu_vn_100). Run the launch 
file below to receive the imu data on ROS topic `/imu/imu` and `/imu/rpy`.

```bash
sudo chmod 666 /dev/ttyUSB0
roslaunch 3D_Slam_tools imu_vn_100.launch
```

### 3) Robot base Odom readings
Get encoder odometry from turtlebot after 'bringup' the robot.
```bash
roslaunch turtlebot_bringup minimal.launch
rostopic echo /odom
```

### Record the bag file!!

After setup, conduct data collection via rosbag with a command
```bash
rosbag record -a
```
When ctrl-c, .bag file will be saved in current working directory


## Run Tools

### 1) 3D SLAM with input velodyne ROS Msg
Here we will have the ROS bag file for a indoor environmenmt. Run with LOAM package with ROS bag file 
(default path is in launch file). User can change the rosbag path and playback setting in the .launch file.
```bash
roslaunch 3D_Slam_tools loam_project.launch
```

To save .pcd and .bt files on fly, Run this on a seperate terminal:
```bash
rosrun 3D_Slam_tools pcd2octomap_node
```

**For tuning, edit config file at `config/param.yaml`.


### 2) PCD Straigtener
When a 3d map is generated, there's a tendency that the map's floor is not exactly normal to the z-axis. Thus, user 
will need to straighten the output .pcd file of a 3D map (by using teleop of turtlebot for convenience sake).

```bash
roslaunch 3D_Slam_tools straightener.launch input_path:="/home/youliang/catkin_ws/input_PC.pcd" output_path:="/home/youliang/catkin_ws/output_octo.bt"
```

User will need to use the arrow keys to control the rotation of the map. Once completed, press ctrl-c it to get the output .bt octomap file.


### 3) 3D Octomap to 2D Occupancy map
Convert .bt file to slices occupancy map .pgm image file. Run map_saver in another terminal to save the map. 
Specify the `PATH_TO_BT_FILE`, optional float value of `Z_MIN` and `Z_MAX`. Both z-value are respected to the velodyne's position.

```bash
roslaunch 3D_Slam_tools octomap_mapping.launch path:=$PATH_TO_BT_FILE z_min:=$Z_MIN  z_max:=$Z_MAX
rosrun map_server map_saver
```

Convert input .pgm to transparent .png map. Then user can use image editting tool to edit the map imagery.

```bash
convert input.pgm  -fuzz 20% -transparent white output.png
```


## Note

###  Odometry Handler Node

If IMU is used, this node will get /imu sensor msg, /point_cloud message, then transform it in a meaningful way to the SLAM node. Currently using vn100 imu for testing.

If encoder odom is used, /odom will be subcribed and publish to /tf, in terms of ` "odom_init"->"camera_init"->"encoder_odom" `.

To run the individual node:
```
rosrun 3D_Slam_tools odom_handler
```

### Troubleshooting

- Compilation on Melodic
  - In melodic, PCL 1.8 is being used, which will resulted in a run time error. (PCL 1.7 is ok)
    - Error Printout: `[multiScanRegistration-2] process has died [pid 19187, exit code -11`
  - make sure that you are using `pcl 1.9`. This issue is mentioned [here](https://github.com/laboshinl/loam_velodyne#troubleshooting).
  - As for Ubuntu 18, PCL 1.9 is not available in debian, thus there's a need to compile from source.
  - Download the PCL 1.9 release: [pcl github release](https://github.com/PointCloudLibrary/pcl/releases)
  - To compile pcl from source, please refer to [here](https://blog.csdn.net/WEICHUAN1107/article/details/87688374)


  - Once the compilation is done, edit these few lines in the `CmakeList` in `loam_velodyne`:
    ```cmake
    find_package(PCL 1.9.1 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    ```
  - Then... Continue with the compilation of LOAM
