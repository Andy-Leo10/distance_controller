# PID distance controller

## Start Simulation

    cd ~/ros2_ws && source install/setup.bash

Empty simulation

    ros2 launch rosbot_xl_gazebo empty_simulation.launch.py

Maze simulation

    ros2 launch rosbot_xl_gazebo simulation.launch.py

## Control test
After compile

    ros2 run distance_controller distance_controller

## Others
Capture points

    ros2 topic echo /rosbot_xl_base_controller/odom --fie pose.pose.position

Manual control

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Visualization