#include "interbotix_xs_sdk/xs_sdk_obj.h"
#include <iostream>
#include <fstream>

/// @brief Class for gripper finger calibration.
class GripperCalibration
{
public:

    /// @brief Construct a new Gripper_calibration object.
    /// @param node_handle - Node object
    /// @param success  - Checker flag if node processes are succesful
    /// @param robot_name  - Name of the robot
    /// @param gripper_name  - Name of the gripper
    GripperCalibration(ros::NodeHandle* node_handle, bool &success, std::string robot_name, std::string gripper_name)
    :node(*node_handle),_gripper_name(gripper_name)
    {
        pub = node.advertise<interbotix_xs_msgs::JointSingleCommand>("/commands/joint_single",1000);
        sub = node.subscribe("/joint_states",1000,&GripperCalibration::calib_callback,this);
        client = node.serviceClient<interbotix_xs_msgs::GripperCalib>("/gripper_calibration");
        calib_complete_srv(0.0);
    }

private:
    ros::NodeHandle node;                                                   // Node object of gripper calibration class
    ros::Publisher pub;                                                     // ROS Publisher instance
    ros::ServiceClient client;                                              // ROS service client instance
    ros::Subscriber sub;                                                    // ROS Subscriber instance
    float prev_val = 0.0;                                                   // Previous gripper value initialization
    interbotix_xs_msgs::JointSingleCommand gripper_command_msg;             // ROS Message for gripper calibration
    interbotix_xs_msgs::GripperCalib gripper_calib_srv;                     // ROS Service for gripper calibration
    std::string _gripper_name;                                              // Name of gripper to be calibrated

    /// @brief Returns the gripper index in joint states of the parsed gripper name
    /// @param names - list of joint names
    /// @return int  - index of the gripper name
    int find_gripper_index(const std::vector<std::string>& names)
    {
        return std::find(names.begin(), names.end(), _gripper_name) - names.begin();
    }

    /// @brief Subscriber callback for performing gripper calibration
    void calib_callback(const sensor_msgs::JointState &msg){
        static int count = 0;
        if(count%40==0){
            int index = find_gripper_index(msg.name);
            float curr_gripper_pos = msg.position[index];
            if(std::abs(curr_gripper_pos-prev_val)>0.0001){
                gripper_command_msg.name = _gripper_name;
                gripper_command_msg.cmd = -200;
                pub.publish(gripper_command_msg);
            }
            else{
                ROS_INFO("The error is 0");
                calib_complete_srv(curr_gripper_pos);
                ros::shutdown();
            }
            prev_val = curr_gripper_pos;
        }
        count++;
    }

    /// @brief ROS Service Client that sends the calibration offset values to SDK
    /// @param min_position_offset - gripper offset value
    void calib_complete_srv(float min_position_offset) {
        gripper_calib_srv.request.gripper_name = _gripper_name;
        gripper_calib_srv.request.offset = min_position_offset;
        bool complete = client.call(gripper_calib_srv);
        if(complete)
        {
            ROS_INFO("Gripper calibration finished successfully");
        }
        else
        {
            ROS_ERROR("Failed to call service Gripper Calibration");
        }
    }

};

/// @brief loads the motor config yaml file and returns a vector of gripper names for calibration
/// @param success - Flag to check if the loading process was succesful
/// @param robot_name - Name of the robot
/// @param yaml_path - Path to the motor config yaml file.
/// @return - [out] vector of gripper names.
std::vector<std::string> load_calibration_config(bool& success, std::string& robot_name, std::string yaml_path){
    YAML::Node yaml_node;
    std::vector<std::string> calibration_joints;
    try
    {
        yaml_node = YAML::LoadFile(yaml_path);
    }
    catch (YAML::BadFile &error)
    {
        ROS_FATAL("[gripper_calib] Calibration Config file was not found or has a bad format. Shutting down...");
        ROS_FATAL("[gripper_calib] YAML Error: '%s'", error.what());
        success = false;
    }
    if (yaml_node.IsNull())
    {
        ROS_FATAL("[gripper_calib] Calibration Config file was not found or has a bad format. Shutting down...");
        success = false;
    }
    YAML::Node all_grippers = yaml_node["grippers"];
    for(const auto gripper : all_grippers)
    {
        calibration_joints.push_back(gripper.first.as<std::string>());
    }
    return calibration_joints;

}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "gripper_calib");
    std::string robot_name, motor_configs_file;
    ros::NodeHandle n1;
    ros::param::get("~motor_configs", motor_configs_file);
    ros::param::get("~robot", robot_name);
    ROS_INFO("Got parameter 1 : %s", robot_name.c_str());
    ROS_INFO("Got parameter 2 : %s",motor_configs_file.c_str());
    bool success = true;
    std::vector<std::string> gripper_order = load_calibration_config(success, robot_name, motor_configs_file);
    for(const auto gripper_name : gripper_order) {
        ROS_INFO("%s",gripper_name.c_str());
        GripperCalibration obj(&n1, success, robot_name, gripper_name);
        ros::Subscriber sub = n1.subscribe("/"+robot_name+"/joint_states",1000,&GripperCalibration::calib_callback,&obj);
        if (success)
            ros::spin();
        else
        {
            ROS_FATAL(
                "[xs_sdk] For troubleshooting, please see "
                "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'");
            continue;
        }
    }
    return 0;
}
