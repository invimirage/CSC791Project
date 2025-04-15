import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rfzhang/ROS-Carla/CSC791Project/lidar_to_carla/install/lidar_to_carla'
