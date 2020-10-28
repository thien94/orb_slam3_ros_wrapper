**Ongoing development.**

# ROS wrapper for ORB-SLAM3

A ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The main idea is to use the ORB-SLAM3 as a standalone library and interface with it instead of putting everything in one package.

- **Pros**: Easy to update the ORB-SLAM3 library (currently in [V0.3 Beta](https://github.com/UZ-SLAMLab/ORB_SLAM3#orb-slam3)). Easy to plug in different variants (and there are many) that are not built for ROS.
- **Cons**: Might be more difficult to spot bugs. Might break when dependencies or upstream changes.


## To-do
- [x] Add nodes for each supported sensor type (mono, mono + imu, stereo, stereo + imu, rgbd).
- [x] Add launch file for EuRoC dataset.
- [ ] Add launch file for real-time hardware (T265 / D435i).
- [ ] Remove Pangolin dependency (require changes in ORB-SLAM3).
- [ ] Add ROS publishers for pose, image, path etc. (require changes in ORB-SLAM3).
- [ ] Test with different variants of ORB-SLAM2/3, such as [DynaSLAM](https://github.com/BertaBescos/DynaSLAM) or [SuperPoint-SLAM](https://github.com/KinglittleQ/SuperPoint_SLAM).

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

- If everything works fine, you can now try the different launch files in the `launch` folder.

## 3. How to run

Example with EuRoC dataset:
```
# In one terminal
roslaunch orb_slam3_ros_wrapper orb_slam3_mono_inertial_euroc.launch
# In another terminal
rosbag play MH_01_easy.bag
```
