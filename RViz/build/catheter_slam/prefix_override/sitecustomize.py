import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oguzhan/Desktop/ros2_ws-20260717T101447Z-1-001/ros2_ws/install/catheter_slam'
