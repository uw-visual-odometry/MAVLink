import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sysop/MAVLink/ros_depth_mav/install/depth_pkg'
