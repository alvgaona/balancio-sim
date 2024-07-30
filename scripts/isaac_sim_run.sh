#!/bin/bash

export isaac_sim_package_path="/home/alvaro/.local/share/ov/pkg/isaac-sim-4.1.0"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export PATH="$PATH:$isaac_sim_package_path/exts/omni.isaac.ros2_bridge/humble/lib"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/omni.isaac.ros2_bridge/humble/lib"
export FASTRTPS_DEFAULT_PROFILES_FILE="./fastdds.xml"

# Run Isaac Sim with ROS2 Bridge Enabled
"$isaac_sim_package_path/isaac-sim.sh" --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge
