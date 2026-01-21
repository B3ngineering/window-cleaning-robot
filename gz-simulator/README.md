# gz-simulator

This is a Gazebo simulator used for testing both physics and software for our window cleaning cable robot.

## Setup

## Relevant Commands

Build with:
```colcon build --packages-select pulley_package```

Launch with:
```ros2 launch pulley_package pulley_gazebo.launch.py```

Move one pulley with:
```ros2 launch pulley_package pulley_gazebo.launch.py```

And of course don't forget to:
```source install/setup.bash```