import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abdelrahman/Wall-Painting-Robot/install/ros2_controllers_test_nodes'
