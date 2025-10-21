import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minirover/ros2_ws/src/minirover/install/minirover_sensores'
