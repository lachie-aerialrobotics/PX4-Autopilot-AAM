# PX4 Drone Autopilot for Aerial Additive Manufacturing
Modified version of PX4 flight stack with additional airframes and gazebo models.
See the [original README](https://github.com/PX4/PX4-Autopilot) for more details.
See also the fork of [PX4-SITL_gazebo-classic](https://github.com/lachie-aerialrobotics/PX4-SITL_gazebo-classic).

# Installation
Tested and working with Ubuntu 20.04 LTS and ROS Noetic.

Clone the repo into your ROS workspace and add the original repo as upstream:
```
cd aam_ws/src
git clone https://github.com/lachie-aerialrobotics/PX4-Autopilot-AAM.git --recursive
cd PX4-Autopilot-AAM
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git pull upstream
```
Run the setup script and install ros dependencies:
```
./Tools/setup/ubuntu.sh
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
```
Install GeographicLib dataset:
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```
Paste the following lines into `~/.bashrc`:
```
source ~/aam_ws/src/PX4-Autopilot-AAM/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/aam_ws/src/PX4-Autopilot-AAM ~/aam_ws/src/PX4-Autopilot-AAM/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/aam_ws/src/PX4-Autopilot-AAM
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/aam_ws/src/PX4-Autopilot-AAM/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/aam_ws/src/PX4-Autopilot-AAM/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```
# To test:
```
roslaunch px4 mavros_posix_sitl.launch vehicle:='hex'
```
# Manipulator simulation
Perform the install instructions at [https://github.com/lachie-aerialrobotics/delta_2](https://github.com/lachie-aerialrobotics/delta_2).
To test the delta manipulator:
```
roslaunch px4 mavros_posix_sitl.launch vehicle:='hex_delta'
```
and the stewart platform:
```
roslaunch px4 mavros_posix_sitl.launch vehicle:='hex_stewart'
```
