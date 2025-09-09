The OnRobot 2FG7 Description package contains XACRO and URDF models for the two-finger gripper from OnRobot. You can use this package to visualise and integrate the hand with your robots. We support [ROS2](https://github.com/touchlab-avatarx/onrobot_2fg7_description).

For examples of how to integrate each device look at `urdf/*.urdf.xacro` files. These attach
the device to a fixed world link for visualisation. Each device is defined as a XACRO
in the corresponding `*.xacro` file. When included a macro for attaching the device to
the robot will be defined. Each macro has a prefix (for uniquely naming all added links and joints),
parent (parent link to attach the device to), and origin (relative offset block).

To see the device in RViz, see example launch
files in `launch/`.
