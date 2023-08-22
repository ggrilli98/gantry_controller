# gantry_controller

# Gantry Controller
ROS 2 Package for the gantry show.

### 1. SSH into gantry and restart the services

    sudo systemctl restart ec_interface.service ec_control.service ec_loadcell.service urg_node.service

#### 2. Sending commands from remote PC
    
    source /opt/ros/foxy/setup.bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run gantry_controller gantry_routine.py 
