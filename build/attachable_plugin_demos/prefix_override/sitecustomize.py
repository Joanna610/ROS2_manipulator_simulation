import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/joanna/ros2_ws/ROS2_manipulator_simulation/install/attachable_plugin_demos'
