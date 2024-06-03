# Description package for the onrobot 2FG7 gripper

## Contents

- rviz config
- launch file
- stl files for visualisation
- ros2 control xacro
- urdf's

## Installation

- git clone this package into the src folder of your ros2 workspace
- run colcon build on your workspace

## Launch

  * Display the gripper with RVIZ:
  ```
  ros2 launch onrobot_2fg7_description rviz.launch
  ```

## Associated packages

- onrobot_api - git@github.com:touchlab-avatarx/onrobot_api.git
- onrobot_robot_driver - git@github.com:touchlab-avatarx/onrobot_robot_driver.git
