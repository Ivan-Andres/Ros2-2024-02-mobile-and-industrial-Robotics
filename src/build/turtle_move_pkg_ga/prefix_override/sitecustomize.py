import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/ros2_ws_2402/src/install/turtle_move_pkg_ga'
