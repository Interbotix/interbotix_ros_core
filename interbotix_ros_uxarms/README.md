# UFactory xArm Driver ROS Packages
![xarm_api_banner](images/xarm_api_banner.png)

## Overview
This folder contains the actual driver package and custom message descriptions needed for controlling the UFactory xArm platforms sold by Interbotix. These packages were copied straight from the [xarm_ros](https://github.com/xArm-Developer/xarm_ros) repository currently maintained by the xArm Developers. The [xarm_driver_node](xarm_api/src/xarm_driver.cpp) was written in C++ and is responsible for publishing joint states and advertising ROS Service Servers to control the arm. The [xarm_ros_client](xarm_api/src/xam_ros_client.cpp) was also written in C++ and presents C++ wrappers around many ROS Service Clients. Thus, there are two ways to control the robot. One is to command the robot via the ROS topics and/or services directly. In this manner, the developer can code in any language that is capable of sending a ROS message. The other approach is to 'skip' the ROS topic layer and use the publicly available functions directly. All the user would need to do is create an instance of the 'XArmROSClient' class as shown [here](xarm_api/src/xarm_ros_client.cpp) to take advantage of these functions.

## Structure

The *xarm_api* package only contains one driver node called **xarm_driver_node**. As mentioned previously, this node is responsible for controlling the physical robot. Please look below for a description of the ROS topics, services, and parameters available to the user.

##### Publishers
- `/<robot_name>/joint_states` - publishes ROS [JointState](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/JointState.html) messages at a set rate of 5 Hz; note that only the 'position' field is valid and is given in radians.
- `/<robot_name>/xarm_states` - publishes custom [RobotMsg](xarm_msgs/msg/RobotMsg.msg) messages describing the control state of the robot. Refer to the message description for details.

##### Subscribers
- `/<robot_name>/sleep_sec` - subscribes to [Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html) messages; it is unclear what this topic is used for...

##### Services
- `/<robot_name>/motion_ctrl` - service to enable/disable the specified joint or all joints; refer to the [SetAxis](xarm_msgs/srv/SetAxis.srv) service description for implementation details.
- `/<robot_name>/set_mode` - service to set the operating mode - can either be 0 (Pose Mode), 1 (Servo Mode), or 2 (Teach Mode); Pose Mode allows the firmware in the Control Box to do motion planning when moving joints; Servo Mode bypasses the internal motion planning in the firmware, which is useful when a user wants to design their own motion planning system; Teach Mode makes it easy to manually move the robot; refer to the [SetInt16](xarm_msgs/srv/SetInt16.srv) service description for implementation details.
- `/<robot_name>/set_state` - service to set the robot state - can either be 0 (Start State), 3 (Pause State), or 4 (Stop State); refer to the [SetInt16](xarm_msgs/srv/SetInt16.srv) service description for implementation details.
- `/<robot_name>/set_tcp_offset` - service to set an end-effector offset relative to the last link on the arm; any Service that moves the end-effector in Cartesian space will take this offset into account; refer to the [TCPOffset](xarm_msgs/srv/TCPOffset.srv) service description for implementation details.
- `/<robot_name>/set_load` - service to set the load seen at the end-effector; refer to the [SetLoad](xarm_msgs/srv/SetLoad.srv) service description for implementation details.
- `/<robot_name>/go_home` - service to make the arm go to its Home Pose (all joints have a value of 0 rad); only works in Mode 0; refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details; note that the 'pose' field should be left blank.
- `/<robot_name>/move_joint` - service to move all the robot's joints to user-specified positions; only works in Mode 0; see more at [5.7 xarm_api/xarm_msgs](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs); refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details.
- `/<robot_name>/move_line` - service to move the end-effector of the arm in a straight line to a specified pose; only works in Mode 0; see more at [5.7 xarm_api/xarm_msgs](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs); refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details.
- `/<robot_name>/move_lineb` - service to move the end-effector of the arm in Cartesian Space through a list of user-specified via points; only works in Mode 0; see more at [5.7 xarm_api/xarm_msgs](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs); refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details.
- `/<robot_name>/move_servoj` - service to move all the robot's joints rapidly (with no motion planning) to user-specified positions; only works in Mode 1; see more at [5.7 xarm_api/xarm_msgs](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs); refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details.
- `/<robot_name>/move_servo_cart` - service to move the end-effector of the arm in a straight line to a specified pose rapidly (with no motion planning); only works in Mode 1; see more at [5.7 xarm_api/xarm_msgs](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs); refer to the [Move](xarm_msgs/srv/Move.srv) service description for implementation details.
- `/<robot_name>/get_err` - service to get the current error state; refer to the [GetErr](xarm_msgs/srv/GetErr.srv) service description for implementation details; note that a value of 0 means that the robot is not in an error state.
- `/<robot_name>/clear_err` - service to reset the error state to 0; refer to the [ClearErr](xarm_msgs/srv/ClearErr.srv) service description for implementation details.
- `/<robot_name>/gripper_config` - service to configure the gripper's speed; refer to the [GripperConfig](xarm_msgs/srv/GripperConfig.srv) service description for implementation details.
- `/<robot_name>/gripper_move` - service to move the gripper; refer to the [GripperMove](xarm_msgs/srv/GripperMove.srv) service description for implementation details.
- `/<robot_name>/gripper_state` - service to get the current gripper's state (range from 0 - 850); refer to the [GripperState](xarm_msgs/srv/GripperState.srv) service description for implementation details.

Note that there are quite a few other services besides these that deal with reading digital or analog pins on the control box and the like. See more info at the [xarm_ros repository](https://github.com/xArm-Developer/xarm_ros#57-xarm_apixarm_msgs). Note that any ROS Service defined there will also work here, but top layer packages (like their version of MoveIt or xArm Planner packages will not).

##### Parameters

- `/<robot_name>/DOF` - degrees of freedom of the arm
- `/<robot_name>/joint_names` - list of joint names for the robot
- `/<robot_name>/xarm_robot_ip` - IP Address of the Control Box; typically 192.168.1.XXX
- `/<robot_name>/wait_for_finish` - For ROS Services that operate in Mode 0, setting this to `True` will make those Services block until the motion is complete


## Usage

To comply with the IRROS standard, the driver package contains no launch or config files (to keep it robot agnostic). The **xarm_driver_node** within it is meant to be included within the launch file located in any *interbotix_XXXXX_control* package (and passed the robot-specific config files). While it's obvious that this node can only be used to control arms, the decision was made to keep the node here and not in the *interbotix_ros_uxarms* sub-repo for consistency purposes. Also, the packages were not modified in any way so that updating them will be much easier.
