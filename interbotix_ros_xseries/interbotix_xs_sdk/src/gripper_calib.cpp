// #include "yaml-cpp/yaml.h"
#include "interbotix_xs_sdk/xs_sdk_obj.hpp"
#include <iostream>
#include <fstream>
#include <memory>

using std::placeholders::_1;

/**
 * @brief This class does the gripper finger calibration and sends the gripper offset to the SDK
 * 
 */
class Gripper_calibration: public rclcpp::Node{
private:
    rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Client<interbotix_xs_msgs::srv::GripperCalib>::SharedPtr client; 
    interbotix_xs_msgs::msg::JointSingleCommand gripper_command_msg;
    float prev_val = 0.0;
    std::vector<std::string> gripper_names;
    std::string robot_name = "px150";
public:
    Gripper_calibration(bool &success): rclcpp::Node("gripper_calib")
    {
        gripper_names = load_calibration_config(success);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/"+robot_name+"/joint_states",1000,std::bind(&Gripper_calibration::calib_callback, this,_1));
        publisher_ = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>("/"+robot_name+"/commands/joint_single",1000);
        client = this->create_client<interbotix_xs_msgs::srv::GripperCalib>("/"+robot_name+"/gripper_calibration");
        calib_complete_srv(0.0,gripper_names[0]);
    }

    /**
     * @brief This function returns the gripper index of the _gripper_name
     * 
     * @param names 
     * @return int 
     */
    int find_gripper_index(const std::vector<std::string>& names)const{
        return std::find(names.begin(),names.end(),gripper_names[0])-names.begin();
    }
    
    /**
     * @brief This is the subscriber callback for performing gripper calibration.
     * 
     * @param msg 
     */
    void calib_callback(const sensor_msgs::msg::JointState &msg){
        static int count = 0;
        if(count%40==0){
            int index = find_gripper_index(msg.name);
            float curr_gripper_pos = msg.position[index];
            RCLCPP_INFO(this->get_logger(),"I heard this position [%f]",msg.position[index]);
            if(std::abs(curr_gripper_pos-prev_val)>0.0001){
                gripper_command_msg.name = gripper_names[0];
                gripper_command_msg.cmd = -200;
                publisher_->publish(gripper_command_msg);
            }
            else{
                RCLCPP_INFO(this->get_logger(),"The error is 0");
                calib_complete_srv(curr_gripper_pos,gripper_names[0]);
                rclcpp::shutdown();
            }
            prev_val = curr_gripper_pos;
        }
        count++;
    }

    /**
     * @brief ROS Service Client that sends the calibration offset values to SDK
     * 
     * @param min_position_offset 
     * @param _gripper_name 
     */
    void calib_complete_srv(float min_position_offset, std::string _gripper_name){
        auto request = std::make_shared<interbotix_xs_msgs::srv::GripperCalib::Request>();
        request->gripper_name = _gripper_name;
        request->offset = min_position_offset;
        auto result = client->async_send_request(request);
        // if (result.get()){
        //     RCLCPP_INFO(this->get_logger(),"Gripper calibration finished successfully");
        // }
        // else{
        //     RCLCPP_ERROR(this->get_logger(),"Failed to call service torque_enable");
        // }
        RCLCPP_INFO(this->get_logger(),"Gripper calibration finished successfully!!");
    }


    /**
     * @brief This function loads the motor config yaml file and returns a vector of gripper names for calibration
     * 
     * @param success 
     * @param robot_name 
     * @param yaml_path 
     * @return std::vector<std::string> 
     */
    std::vector<std::string> load_calibration_config(bool& success){
        // this->declare_parameter<std::string>("motor_configs", "");
        // this->get_parameter("motor_configs", yaml_path);
        std::string yaml_path = "/home/pj/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/px150.yaml";
        YAML::Node yaml_node;
        std::vector<std::string> calibration_joints;
        try
        {
            yaml_node = YAML::LoadFile(yaml_path);
        }
        catch (YAML::BadFile &error)
        {
            RCLCPP_FATAL(this->get_logger(),"[xs_sdk] Calibration Config file was not found or has a bad format. Shutting down...");
            RCLCPP_FATAL(this->get_logger(),"[xs_sdk] YAML Error: '%s'", error.what());
            success = false;
        }
        if (yaml_node.IsNull())
        {
            RCLCPP_FATAL(this->get_logger(),"[xs_sdk] Calibration Config file was not found. Shutting down...");
            success = false;
        }
        YAML::Node all_grippers = yaml_node["grippers"];
        for(const auto gripper:all_grippers){
            calibration_joints.push_back(gripper.first.as<std::string>());
        }
        return calibration_joints;
                
    }

};





int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool success = true;

  auto node = std::make_shared<Gripper_calibration>(success);
  if (success) {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } else {
    RCLCPP_FATAL(
      interbotix_xs::LOGGER,
      "For troubleshooting, please see "
      "'https://docs.trossenrobotics.com/interbotix_xsarms_docs/troubleshooting.html'.");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}