**Ongoing development.**

# ROS wrapper for ORB-SLAM3

A ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The main idea is to use the ORB-SLAM3 as a standalone library and interface with it instead of putting everything in one package.

- **Pros**:
  - Easy to update the ORB-SLAM3 library (currently in [V0.3 Beta](https://github.com/UZ-SLAMLab/ORB_SLAM3#orb-slam3)).
  - Easy to plug in different variants (and there are many) that are not built for ROS (hopefully).
- **Cons**:
  - Might be more difficult to spot bugs.
  - Development involves more steps (1. Make changes in ORB-SLAM3 library -> 2. Build ORB-SLAM3 -> 3. Change the ROS-wrapper if necessary -> 4. Test).
  - Might break when dependencies or upstream changes.


## To-do
- [x] Add nodes for each supported sensor type (mono, mono + imu, stereo, stereo + imu, rgbd).
- [x] Add launch files for the nodes.
- [x] Add ROS publishers for pose, pointcloud, path.
- [ ] Add ROS publisher for tracking image (require changes in ORB-SLAM3 to be merged).
- [ ] Test with different variants of ORB-SLAM2/3, such as [DynaSLAM](https://github.com/BertaBescos/DynaSLAM) or [SuperPoint-SLAM](https://github.com/KinglittleQ/SuperPoint_SLAM).

## Current known limitations
- For mono+imu and stereo+imu cases, the world frame seems to change arbitrarily each time a big Bundle Adjustment is invoked. As a result, you will see the pose data suddenly jumps around, but actually it is the world frame that teleports. The issue is raised [here](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/172).  
- Currently there is no public API to obtain the tracking image from ORB-SLAM3, so wait for [this PR](https://github.com/UZ-SLAMLab/ORB_SLAM3/pull/174) to be merged.
- Currently there is no public API to obtain the full map points and other data to debug.

# Installation
First install ORB-SLAM3 normally with all of its dependencies (any location is fine) then install this package in a ```catkin build``` environment.

## 1. ORB-SLAM3 (original or other variants)

- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites).
- Build and install ORB-SLAM3. Any location is fine (default directory that I use later on is the home folder `~`):
```
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check the issue list from the [original repo](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues) and rebuild the package.

## 2. orb_slam3_ros_wrapper (this package)

- Clone the package. Note that it should be a `catkin build` workspace.
```
cd ~/catkin_ws/src/ # Or the name of your workspace
git clone https://github.com/thien94/orb_slam3_ros_wrapper.git
```

- Open `CMakeLists.txt` and change the directory that leads to ORB-SLAM3 library at the beginning of the file (default is home folder)
```
cd ~/catkin_ws/src/orb_slam3_ros_wrapper/
nano CMakeLists.txt

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
)
```

- Build the package normally.
```
cd ~/catkin_ws/
catkin build
```

- Unzip the `ORBvoc.txt` file in the `config` folder in this package. Alternatively, you can change the `voc_file` param in the launch file to point to the right folder.
```
cd ~/catkin_ws/src/orb_slam3_ros_wrapper/config
tar -xf ORBvoc.txt.tar.gz
```

- Install `hector-trajectory-server` to visualize the trajectory.
```
sudo apt install ros-[kinetic/melodic]-hector-trajectory-server
```

- If everything works fine, you can now try the different launch files in the `launch` folder.

## 3. How to run

Example with EuRoC dataset:
```
# In one terminal
roslaunch orb_slam3_ros_wrapper orb_slam3_mono_inertial_euroc.launch
# In another terminal
rosbag play MH_01_easy.bag
```
Similarly for other sensor types.

# Topics
The following topics are published by each node:
- `/orb_slam3_ros/map_points` ([`PointCloud2`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) containing all keypoints being tracked.
- `/orb_slam3_ros/camera` ([`PoseStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseStamped.html)): current pose of the camera in the world frame, as returned by ORB-SLAM3 with the world coordinate transformed to conform the ROS standard.
- `tf`: transformation from the camera fraame to the world frame.