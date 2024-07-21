$env:isaac_sim_package_path = "D:\.ov\pkg\isaac-sim-2023.1.1"

$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
$env:PATH = "$env:PATH;$env:isaac_sim_package_path\exts\omni.isaac.ros2_bridge\humble\lib"
$env:LD_LIBRARY_PATH = "$env:LD_LIBRARY_PATH;$env:isaac_sim_package_path\exts\omni.isaac.ros2_bridge\humble\lib"

$env:FASTRTPS_DEFAULT_PROFILES_FILE = "./fastdds.xml"

# Run Isaac Sim with ROS2 Bridge Enabled
& "$env:isaac_sim_package_path\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=omni.isaac.ros2_bridge
