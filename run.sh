source ./install/setup.bash
export ROS_DOMAIN_ID=$(expr $1 + 1)
ros2 launch rm_serial_driver serial_driver.launch.py
