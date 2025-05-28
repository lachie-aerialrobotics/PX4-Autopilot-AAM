<<<<<<< HEAD
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
# Test
Check the iris quad builds:
```
make px4_sitl gazebo-classic
```
then:
```
roslaunch px4 mavros_posix_sitl.launch vehicle:='hex'
```
# Worlds
Models of the Imperial College flight arena and the Empa DroneHub are also included. Try:
```
make px4_sitl gazebo-classic_hex_ARLarena
```
```
make px4_sitl gazebo-classic_hex_DroneHub
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
=======
# PX4 Drone Autopilot

[![Releases](https://img.shields.io/github/release/PX4/PX4-Autopilot.svg)](https://github.com/PX4/PX4-Autopilot/releases) [![DOI](https://zenodo.org/badge/22634/PX4/PX4-Autopilot.svg)](https://zenodo.org/badge/latestdoi/22634/PX4/PX4-Autopilot)

[![Build Targets](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml/badge.svg?branch=main)](https://github.com/PX4/PX4-Autopilot/actions/workflows/build_all_targets.yml) [![SITL Tests](https://github.com/PX4/PX4-Autopilot/workflows/SITL%20Tests/badge.svg?branch=master)](https://github.com/PX4/PX4-Autopilot/actions?query=workflow%3A%22SITL+Tests%22)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This repository holds the [PX4](http://px4.io) flight control solution for drones, with the main applications located in the [src/modules](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules) directory. It also contains the PX4 Drone Middleware Platform, which provides drivers and middleware to run drones.

PX4 is highly portable, OS-independent and supports Linux, NuttX and MacOS out of the box.

* Official Website: http://px4.io (License: BSD 3-clause, [LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE))
* [Supported airframes](https://docs.px4.io/main/en/airframes/airframe_reference.html) ([portfolio](https://px4.io/ecosystem/commercial-systems/)):
  * [Multicopters](https://docs.px4.io/main/en/frames_multicopter/)
  * [Fixed wing](https://docs.px4.io/main/en/frames_plane/)
  * [VTOL](https://docs.px4.io/main/en/frames_vtol/)
  * [Autogyro](https://docs.px4.io/main/en/frames_autogyro/)
  * [Rover](https://docs.px4.io/main/en/frames_rover/)
  * many more experimental types (Blimps, Boats, Submarines, High Altitude Balloons, Spacecraft, etc)
* Releases: [Downloads](https://github.com/PX4/PX4-Autopilot/releases)

## Releases

Release notes and supporting information for PX4 releases can be found on the [Developer Guide](https://docs.px4.io/main/en/releases/).

## Building a PX4 based drone, rover, boat or robot

The [PX4 User Guide](https://docs.px4.io/main/en/) explains how to assemble [supported vehicles](https://docs.px4.io/main/en/airframes/airframe_reference.html) and fly drones with PX4. See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Changing Code and Contributing

This [Developer Guide](https://docs.px4.io/main/en/development/development.html) is for software developers who want to modify the flight stack and middleware (e.g. to add new flight modes), hardware integrators who want to support new flight controller boards and peripherals, and anyone who wants to get PX4 working on a new (unsupported) airframe/vehicle.

Developers should read the [Guide for Contributions](https://docs.px4.io/main/en/contribute/).
See the [forum and chat](https://docs.px4.io/main/en/#getting-help) if you need help!


## Weekly Dev Call

The PX4 Dev Team syncs up on a [weekly dev call](https://docs.px4.io/main/en/contribute/).

> **Note** The dev call is open to all interested developers (not just the core dev team). This is a great opportunity to meet the team and contribute to the ongoing development of the platform. It includes a QA session for newcomers. All regular calls are listed in the [Dronecode calendar](https://www.dronecode.org/calendar/).


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

For the latest stats on contributors please see the latest stats for the Dronecode ecosystem in our project dashboard under [LFX Insights](https://insights.lfx.linuxfoundation.org/foundation/dronecode). For information on how to update your profile and affiliations please see the following support link on how to [Complete Your LFX Profile](https://docs.linuxfoundation.org/lfx/my-profile/complete-your-lfx-profile). Dronecode publishes a yearly snapshot of contributions and achievements on its [website under the Reports section](https://dronecode.org).

## Supported Hardware

For the most up to date information, please visit [PX4 User Guide > Autopilot Hardware](https://docs.px4.io/main/en/flight_controller/).

## Project Governance

The PX4 Autopilot project including all of its trademarks is hosted under [Dronecode](https://www.dronecode.org/), part of the Linux Foundation.

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>
>>>>>>> upstream/main
