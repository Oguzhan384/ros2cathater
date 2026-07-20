import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oguzhan/Desktop/ros2_ws-20260717T101447Z-1-001 (2)/ros2_ws/src/catheter_slam/install/catheter_slam'
