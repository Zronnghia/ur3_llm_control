#!/usr/bin/env bash
# Fetch the Humble sources needed to rebuild the real UR URDF offline.
# On a machine with ROS 2 Humble installed you do NOT need this: just run
#   python3 verify_urdf_fk.py --from-ros
set -e
cd "$(dirname "$0")" && mkdir -p _deps && cd _deps
clone() { [ -d "$2" ] || git clone -q --depth 1 -b "$3" "https://github.com/$1.git" "$2"; }
clone UniversalRobots/Universal_Robots_ROS2_Description ur_description humble
clone UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation ur_simulation_gazebo humble
clone ros/xacro xacro 2.0.8
echo "deps ready in $(pwd)"
