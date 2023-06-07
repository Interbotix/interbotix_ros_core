#include "interbotix_xs_sdk/xs_sdk_obj.h"
#include <iostream>
#include <fstream>
/**
 * @brief This class does the gripper finger calibration and sends the gripper offset to the SDK
 * 
 */
class Gripper_calibration{
public:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::ServiceClient client;
    ros::Subscriber sub;
    float prev_val = 0.0;
    interbotix_xs_msgs::JointSingleCommand gripper_command_msg;
    interbotix_xs_msgs::GripperCalib gripper_calib_srv;
    YAML::Node yaml_node;
    std::string yaml_path;
    std::vector<std::string> calibration_joints;
    std::vector<float> calibration_offsets;
    std::string _gripper_name;

    Gripper_calibration(ros::NodeHandle* node_handle, bool &success, std::string robot_name, std::string gripper_name):node(*node_handle),_gripper_name(gripper_name)
    {
        pub = node.advertise<interbotix_xs_msgs::JointSingleCommand>("/"+robot_name+"/commands/joint_single",1000);
        sub = node.subscribe("/"+robot_name+"/joint_states",1000,&Gripper_calibration::calib_callback,this);
        client = node.serviceClient<interbotix_xs_msgs::GripperCalib>("/"+robot_name+"/gripper_calibration");
        calib_complete_srv(0.0);
        load_calibration_config(success, robot_name);

        // gripper_command_msg.name = "gripper";
        // gripper_command_msg.cmd = 0;
        // pub.publish(gripper_command_msg); 
    }

    void load_calibration_config(bool& success, std::string& robot_name){
        // ros::param::get("~calibration_config", yaml_path);
        yaml_path = "/home/pinak/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/px150.yaml";
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
        for(const auto gripper:all_grippers){
            calibration_joints.push_back(gripper.first.as<std::string>());
        }
                
    }

    int find_gripper_index(const std::vector<std::string>& names){
        return std::find(names.begin(),names.end(),_gripper_name)-names.begin();
    }
    
    void calib_callback(const sensor_msgs::JointState &msg){
        static int count = 0;
        if(count%30==0){
            int index = find_gripper_index(msg.name);
            float curr_gripper_pos = msg.position[index];
            ROS_INFO("I heard this position [%f]",msg.position[index]);
            if(std::abs(curr_gripper_pos-prev_val)>0.0001){
                gripper_command_msg.name = _gripper_name;
                gripper_command_msg.cmd = -140;
                pub.publish(gripper_command_msg);
            }
            else{
                ROS_INFO("The error is 0");
                calib_complete_srv(curr_gripper_pos);
                ros::shutdown();
            }
            // ROS_INFO("Error:%f",curr_gripper_pos-prev_val);
            prev_val = curr_gripper_pos;
        }
        count++;
    }

    void calib_complete_srv(float min_position_offset){
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


int main( int argc, char** argv )
{
    ros::init(argc, argv, "gripper_calib");
    std::string param1,param2;
    ros::NodeHandle n1("~");
    n1.getParam("robot",param1);
    n1.getParam("gripper_name",param2);
    ROS_INFO("Got parameter : %s", param1.c_str());
    ROS_INFO("Got parameter2 : %s",param2.c_str());
    bool success = true;
    Gripper_calibration obj(&n1,success,param1,param2);
    ros::Subscriber sub = n1.subscribe("/"+param1+"/joint_states",1000,&Gripper_calibration::calib_callback,&obj);
    if (success)
        ros::spin();
    else
    {
        ROS_FATAL(
            "[xs_sdk] For troubleshooting, please see "
            "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'");
        ros::shutdown();
    }
    return 0;
}
