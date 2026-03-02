# ROS README

Message contracts, nodes, systems.

## Sensor Node

- Reads from STM32 board 1 that is connected to ultrasonic sensors

## Vision Node

- Reads from the camera connected to Pi, then uses OpenCV for inference
- Determines whether section is clean or dirty and sends that to the navigation node

## Navigation Node

- Takes user input as a target position, and sends this target position to the motion control node
- Determines when to clean

## Motion Controller Node

- Subscribes to sensor and navigation nodes
- Does cable math and sends motor commands to STM bridge for traversal and cleaning

## Motor STM32 Bridge Node

- Takes motor commands and translates them to serial for communication to STM board via USB
- Should be able to take serial data from motor drivers and report motor statuses to the motion control node for use in calculations

## Sensor STM32 Bridge Node

- Takes sensor data from ultrasonic sensor array
- When an obstacle is detected, sends emergency stop to navigation node

## Topic Registry

| Topic | Publisher | Subscribers | Message Type |
| --- | --- | --- | --- |
| /target_position | navigation_node | motion_controller | gemoetry_msgs/Point |
| /motor_commands | motion_controller | motor_stm32_bridge | window_cleaner/MotorCommand |
| /joint_states | motor_stm32_bridge | motio_controller | sensor_msgs/JointState |
| /robot_state | motion_controller | navigation_node | window_cleaner/RobotState |
| /cleaning_result | vision_node | motion_controller | window_cleaner/CleaningResult |
| capture_request | navigation_node | vision_node | std_msgs/Bool |
| /camera/image_raw | vision_node | NA | sensor_msgs/Image |
| /emergency_stop | sensor_stm32_bridge | motion_controller | window_cleaner/Stop |