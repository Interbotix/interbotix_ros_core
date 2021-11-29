cmake_minimum_required(VERSION 2.8.3)
project(interbotix_xs_msgs)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    std_msgs
    trajectory_msgs
    geometry_msgs
)

add_message_files(
    FILES
    JointGroupCommand.msg
    JointSingleCommand.msg
    JointTrajectoryCommand.msg
    # xsarm_diagnostic_tool
    JointTemps.msg
    # xsarm_joy
    ArmJoy.msg
    #xshexapod_joy
    HexJoy.msg
    #xslocobot_joy
    LocobotJoy.msg
    #xsturret_joy
    TurretJoy.msg
)

add_service_files(
    FILES
    Reboot.srv
    RobotInfo.srv
    MotorGains.srv
    TorqueEnable.srv
    OperatingModes.srv
    RegisterValues.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    trajectory_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    trajectory_msgs
)
