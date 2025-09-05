import sys
if sys.prefix == '/Users/yokoyama/Development/ros2/.pixi/envs/default':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/yokoyama/Development/ros2/install/test_node'
