#include "string.h"

#include "rclcpp/rclcpp.hpp"
#include "epos_msgs/msg/control_mode.hpp"
#include "epos_msgs/msg/throttle.hpp"
#include "epos_msgs/msg/brake.hpp"
#include "epos_msgs/msg/steering_wheel.hpp"

class ModeControl : public rclcpp::Node
{
  public:
    ModeControl() : Node("mode_control")
    {
      mode_publisher_ = this->create_publisher<epos_msgs::msg::ControlMode>("mode", 100);
      pos_throttle_publisher_ = this->create_publisher<epos_msgs::msg::Throttle>("ctrl_mode/throttle", 100);
      pos_brake_publisher_ = this->create_publisher<epos_msgs::msg::Brake>("ctrl_mode/brake", 100);
      pos_steering_wheel_publisher_ = this->create_publisher<epos_msgs::msg::SteeringWheel>("ctrl_mode/steer", 100);
      
      processing_thread_ = std::thread(&ModeControl::process_commands, this);
    }

    private:
      void process_commands()
      {
        while(rclcpp::ok())
        {
          std::string command;

          std::cout << "Escribe comando: ";
          std::getline(std::cin, command);

          if(*(command.c_str()) == 'M')
          {
            auto mode_msg = epos_msgs::msg::ControlMode();
            mode_msg.mode = atoi(command.c_str()+1);

            rclcpp::Clock time;
            mode_msg.header.stamp = time.now();
            mode_publisher_->publish(mode_msg);
          }
          if(*(command.c_str()) == 'A')
          {
            auto mode_msg = epos_msgs::msg::Throttle();
            mode_msg.throttle = atof(command.c_str()+1);

            rclcpp::Clock time;
            mode_msg.header.stamp = time.now();
            pos_throttle_publisher_->publish(mode_msg);
          }
          if(*(command.c_str()) == 'F')
          {
            auto mode_msg = epos_msgs::msg::Brake();
            mode_msg.brake = atof(command.c_str()+1);

            rclcpp::Clock time;
            mode_msg.header.stamp = time.now();
            pos_brake_publisher_->publish(mode_msg);
          }
          if(*(command.c_str()) == 'V')
          {
            auto mode_msg = epos_msgs::msg::SteeringWheel();
            mode_msg.steer = atof(command.c_str()+1);

            rclcpp::Clock time;
            mode_msg.header.stamp = time.now();
            pos_steering_wheel_publisher_->publish(mode_msg);
          }
        }
      }

    rclcpp::Publisher<epos_msgs::msg::ControlMode>::SharedPtr mode_publisher_;
    rclcpp::Publisher<epos_msgs::msg::Throttle>::SharedPtr pos_throttle_publisher_;
    rclcpp::Publisher<epos_msgs::msg::Brake>::SharedPtr pos_brake_publisher_;
    rclcpp::Publisher<epos_msgs::msg::SteeringWheel>::SharedPtr pos_steering_wheel_publisher_;
    std::thread processing_thread_; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeControl>());
  rclcpp::shutdown();
  return 0;
}