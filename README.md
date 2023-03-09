# PX4 Drone Autopilot for Aerial Additive Manufacturing
Modified version of PX4 flight stack with additional airframes and gazebo models.
See the [original README](https://github.com/PX4/PX4-Autopilot) for more details.
See also the fork of [PX4-SITL_gazebo-classic](https://github.com/lachie-aerialrobotics/PX4-SITL_gazebo-classic).

# Installation
Clone the repo into your ROS workspace and add the original repo as upstream:
```
cd catkin_ws/src
git clone https://github.com/lachie-aerialrobotics/PX4-Autopilot-AAM.git --recursive
cd PX4-Autopilot-AAM
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
```
Paste the following lines into `~/.bashrc`:
```
source ~/aam_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/aam_ws/src/PX4-Autopilot ~/aam_ws/src/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/aam_ws/src/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/aam_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/aam_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
```
To test:
```
make px4_sitl gazebo-classic_hex
```
