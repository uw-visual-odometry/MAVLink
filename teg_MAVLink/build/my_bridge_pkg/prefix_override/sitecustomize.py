import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sysop/MAVLink/teg_MAVLink/install/my_bridge_pkg'
