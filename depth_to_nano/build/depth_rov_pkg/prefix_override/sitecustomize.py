import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sysop/MAVLink/depth_to_nano/install/depth_rov_pkg'
