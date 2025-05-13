import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/johan/Dev/graduation_project/ros_ws/install/ros_serail2wifi'
