# ROS README

Message contracts, nodes, systems.

## Architecture

### Vision Node

- Reads from the camera connected to Pi, then uses OpenCV for inference
- Determines whether section is clean or dirty and sends that to the navigation node

### Navigation Node

- Takes user input as a target position, and sends this target position to the motion control node
- Determines when to clean

### Motion Controller Node

- Subscribes to sensor and navigation nodes
- Does cable math and sends motor commands to STM bridge for traversal and cleaning

### Motor STM32 Bridge Node

- Takes motor commands and translates them to serial for communication to STM board via USB
- Should be able to take serial data from motor drivers and report motor statuses to the motion control node for use in calculations

### Sensor STM32 Bridge Node

- Takes sensor data from ultrasonic sensor array
- When an obstacle is detected, sends emergency stop to navigation node

### Topic Registry

| Topic | Publisher | Subscribers | Message Type |
| --- | --- | --- | --- |
| /target_position | navigation_node | motion_controller | gemoetry_msgs/Point |
| /motor_commands | motion_controller | motor_stm32_bridge | window_cleaner/MotorCommand |
| /joint_states | motor_stm32_bridge | motion_controller | sensor_msgs/JointState |
| /robot_state | motion_controller | navigation_node | window_cleaner/RobotState |
| /cleaning_result | vision_node | navigation_node | window_cleaner/CleaningResult |
| /capture_request | navigation_node | vision_node | std_msgs/Bool |
| /camera/image_raw | vision_node | NA | sensor_msgs/Image |
| /emergency_stop | sensor_stm32_bridge | motion_controller | window_cleaner/Stop |

## Setup and Execution

First, ensure that `roscore` is running. Use `hostname -I` to get the IP address, then update `~/.bashrc` with the correct ip and source it for this to run.
After cloning, run `catkin_make` in the `ros1_ws` directory and then `source devel/setup.bash`. If you get chmod errors, we can make scripts executable with `chmod +x ~/ros1_ws/src/window-cleaning-robot/ros_package/nodes/*.py`. 
Finally, run the sim with `roslaunch window_cleaner sim.launch`. Verify it's working a little bit with `rostopic echo /emergency_stop`.
