**Ongoing development.**

# ROS wrapper for ORB-SLAM3

My attempt at providing a ROS wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). The main idea is to use the ORB-SLAM3 as a standalone library and interface with it instead of putting everything in one package.

- **Pros**: Easy to update the ORB-SLAM3 library (currently in [V0.3 Beta](https://github.com/UZ-SLAMLab/ORB_SLAM3#orb-slam3)). Easy to try different variants of ORB (and there are many) that are not built for ROS.
- **Cons**: Might be more difficult to spot bugs. Might break when dependencies or upstream changes.


## To-do
- [x] Add nodes for each supported sensor type (mono, mono + imu, stereo, stereo + imu, rgbd).
- [x] Add launch file for EuRoC dataset.
- [ ] Add launch file for real-time hardware (T265 / D435i).
- [ ] Add pose publisher and remove Pandolin dependency (require changes in ORB-SLAM3).

# Installation
First install ORB-SLAM3 normally with all of its dependencies (any location is fine) then install this package in a ```catkin build``` environment.

## 1. ORB-SLAM3 (original or other variant)
- Install the [prerequisites](https://github.com/UZ-SLAMLab/ORB_SLAM3#2-prerequisites).
- Build and install ORB-SLAM3. Any location is fine (default directory that I use later on is the home folder `~`)
```
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```
- Make sure that **`libORB_SLAM3.so`** is created in the *ORB_SLAM3/lib* folder. If not, check and rebuild the package.

## 2. orb-slam3-ros-wrapper (this package)
- Open `CMakeLists.txt` and change the directory that leads to ORB-SLAM3 library at the beginning of the file (default is home folder)
```
# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   $ENV{HOME}/ORB_SLAM3
)
```
- Clone and build the package. Note that it should be a `catkin build` workspace.

```
cd catkin_ws/src
git clone https://github.com/thien94/orb-slam3-ros-wrapper.git
cd ../
catkin build
```
- Copy the `ORBvoc.txt` file from `ORB-SLAM3/Vocabulary/` folder to the `config` folder in this package. Alternatively, you can change the `voc_file` to point to the right folder.

- If everything works fine, try the different launch files in the `launch` folder.

## 3. How to run
Example with EuRoC dataset:
```
# In one terminal
roslaunch orb_slam3_ros_wrapper orb_slam3_mono_inertial_euroc.launch
# In another terminal
rosbag play MH_01_easy.bag
```
