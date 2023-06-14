#include "interbotix_xs_sdk/xs_sdk_obj.h"
#include <iostream>
#include <fstream>
/**
 * @brief Class for gripper finger calibration.
 */
class Gripper_calibration{
private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::ServiceClient client;
    ros::Subscriber sub;
    float prev_val = 0.0;
    interbotix_xs_msgs::JointSingleCommand gripper_command_msg;
    interbotix_xs_msgs::GripperCalib gripper_calib_srv;
    std::string _gripper_name;
public:
    /**
     * @brief Construct a new Gripper_calibration object.
     * @param node_handle - Node object
     * @param success  - Checker flag if node processes are succesful
     * @param robot_name  - Name of the robot
     * @param gripper_name  - Name of the gripper
     */
    Gripper_calibration(ros::NodeHandle* node_handle, bool &success, std::string robot_name, std::string gripper_name):node(*node_handle),_gripper_name(gripper_name)
    {
        pub = node.advertise<interbotix_xs_msgs::JointSingleCommand>("/"+robot_name+"/commands/joint_single",1000);
        sub = node.subscribe("/"+robot_name+"/joint_states",1000,&Gripper_calibration::calib_callback,this);
        client = node.serviceClient<interbotix_xs_msgs::GripperCalib>("/"+robot_name+"/gripper_calibration");
        calib_complete_srv(0.0,_gripper_name);
    }

    /**
     * @brief Returns the gripper index in joint states of the parsed gripper name
     * @param names - list of joint names
     * @return int  - index of the gripper name
     */
    int find_gripper_index(const std::vector<std::string>& names){
        return std::find(names.begin(),names.end(),_gripper_name)-names.begin();
    }

    /**
     * @brief Subscriber callback for performing gripper calibration
     */
    void calib_callback(const sensor_msgs::JointState &msg){
        static int count = 0;
        if(count%40==0){
            int index = find_gripper_index(msg.name);
            float curr_gripper_pos = msg.position[index];
            ROS_INFO("I heard this position [%f]",msg.position[index]);
            if(std::abs(curr_gripper_pos-prev_val)>0.0001){
                gripper_command_msg.name = _gripper_name;
                gripper_command_msg.cmd = -200;
                pub.publish(gripper_command_msg);
            }
            else{
                ROS_INFO("The error is 0");
                calib_complete_srv(curr_gripper_pos,_gripper_name);
                ros::shutdown();
            }
            prev_val = curr_gripper_pos;
        }
        count++;
    }

    /**
     * @brief ROS Service Client that sends the calibration offset values to SDK
     * @param min_position_offset - gripper offset value
     * @param _gripper_name - gripper name
     */
    void calib_complete_srv(float min_position_offset, std::string _gripper_name){
        gripper_calib_srv.request.gripper_name = _gripper_name;
        gripper_calib_srv.request.offset = min_position_offset;
        bool complete = client.call(gripper_calib_srv);
        if(complete){
            ROS_INFO("Gripper calibration finished successfully");
        }
        else{
            ROS_ERROR("Failed to call service torque_enable");
        }
    }

};


/**
 * @brief loads the motor config yaml file and returns a vector of gripper names for calibration
 * @param success - Flag to check if the loading process was succesful
 * @param robot_name - Name of the robot
 * @param yaml_path - Path to the motor config yaml file.
 * @return - [out] vector of gripper names.
 */
std::vector<std::string> load_calibration_config(bool& success, std::string& robot_name, std::string yaml_path){
    YAML::Node yaml_node;
    std::vector<std::string> calibration_joints;
    try
    {
        yaml_node = YAML::LoadFile(yaml_path);
    }
    catch (YAML::BadFile &error)
    {
        ROS_FATAL("[xs_sdk] Calibration Config file was not found or has a bad format. Shutting down...");
        ROS_FATAL("[xs_sdk] YAML Error: '%s'", error.what());
        success = false;
    }
    if (yaml_node.IsNull())
    {
        ROS_FATAL("[xs_sdk] Calibration Config file was not found. Shutting down...");
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
    std::string param1,param2,motor_configs_file;
    ros::NodeHandle n1;
    ros::param::get("~motor_configs", motor_configs_file);
    ros::param::get("~robot", param1);
    ROS_INFO("Got parameter 1 : %s", param1.c_str());
    ROS_INFO("Got parameter 2 : %s",motor_configs_file.c_str());
    bool success = true;
    std::vector<std::string> gripper_names = load_calibration_config(success,param1,motor_configs_file);
    for(const auto name:gripper_names){
        ROS_INFO("%s",name.c_str());
        Gripper_calibration obj(&n1,success,param1,name);
        ros::Subscriber sub = n1.subscribe("/"+param1+"/joint_states",1000,&Gripper_calibration::calib_callback,&obj);
        if (success)
            ros::spin();
        else
        {
            ROS_FATAL(
                "[xs_sdk] For troubleshooting, please see "
                "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'");
            // ros::shutdown();
            continue;
        }
    }
    return 0;
}
