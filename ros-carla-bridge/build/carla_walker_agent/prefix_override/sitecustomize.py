import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rfzhang/ROS-Carla/CSC791Project/ros-carla-bridge/install/carla_walker_agent'
