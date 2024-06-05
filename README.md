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

## Implementation

- There are two separate macro's in the xacro file
  - 1 for the base - *onrobot_2fg7*
  - 1 for the fingertips - *onrobot_2fg7_fingertip*

The same macro is used for both fingertips in conjunction with a prefix to differentiate them.
Each fingertip macro is attached to the left and right_attachment links respectively.

If you use a different macro in the urdf.xacro remember to rotate the right fingertip by "pi"
around the z-axis to achieve the correct orientation.

Example:
```
<!-- Attach right fingertip -->
    <xacro:onrobot_2fg7_fingertip prefix="right_" parent="$(arg prefix)right_attachment">
        <origin xyz="0 0 0" rpy="0 0 3.14159265359"/>
    </xacro:onrobot_2fg7_fingertip>

    <!-- Attach left fingertip -->
    <xacro:onrobot_2fg7_fingertip prefix="left_" parent="$(arg prefix)left_attachment">
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:onrobot_2fg7_fingertip>
```

The attachment location is is no longer outwards or inwards, instead it is centred on the 
mounting bolts

## Launch

  * Display the gripper with RVIZ:
  ```
  ros2 launch onrobot_2fg7_description rviz.launch
  ```

## Associated packages

- onrobot_api - git@github.com:touchlab-avatarx/onrobot_api.git
- onrobot_robot_driver - git@github.com:touchlab-avatarx/onrobot_robot_driver.git
