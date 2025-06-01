import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/singh/ros2_sensor_ws/install/yolov5_ros'
