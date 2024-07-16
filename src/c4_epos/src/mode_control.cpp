#include "rclcpp/rclcpp.hpp"
#include "epos_msgs/msg/control_mode.hpp"
#include "epos_msgs/msg/throttle.hpp"
#include "epos_msgs/msg/brake.hpp"
#include "epos_msgs/msg/steering_wheel.hpp"
#include <chrono>
#include <string>


class ModeControl : public rclcpp::Node
{
  public:
  bool publish_th = true;
    ModeControl() : Node("mode_control"), last_mode_(0), last_throttle_(0.0), last_brake_(0.0), last_steer_(0.0)
    {
      mode_publisher_ = this->create_publisher<epos_msgs::msg::ControlMode>("mode", 100);
      pos_throttle_publisher_ = this->create_publisher<epos_msgs::msg::Throttle>("ctrl_mode/throttle", 100);
      pos_brake_publisher_ = this->create_publisher<epos_msgs::msg::Brake>("ctrl_mode/brake", 100);
      pos_steering_wheel_publisher_ = this->create_publisher<epos_msgs::msg::SteeringWheel>("ctrl_mode/steer", 100);
      
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ModeControl::publish_commands, this));

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
            last_mode_ = atoi(command.c_str() + 1);
          
          if(*(command.c_str()) == 'A')
            last_throttle_ = atof(command.c_str() + 1);

          if(*(command.c_str()) == 'F')
            last_brake_ = atof(command.c_str()+1);
          
          if(*(command.c_str()) == 'V')
            last_steer_ = atof(command.c_str()+1);

          if(command == "stop")
          {
            publish_th = false;
          }
        }
      }

      void publish_commands()
      {
        rclcpp::Clock time;

        epos_msgs::msg::ControlMode mode_msg;
        mode_msg.mode = last_mode_;
        mode_msg.header.stamp = time.now();
        //if(publish_th == true)
          mode_publisher_->publish(mode_msg);

        epos_msgs::msg::Throttle throttle_msg;
        throttle_msg.throttle = last_throttle_;
        throttle_msg.header.stamp = time.now();
        //if(publish_th == true)
          pos_throttle_publisher_->publish(throttle_msg);

        epos_msgs::msg::Brake brake_msg;
        brake_msg.brake = last_brake_;
        brake_msg.header.stamp = time.now();
        //if(publish_th == true)
          pos_brake_publisher_->publish(brake_msg);

        epos_msgs::msg::SteeringWheel steering_wheel_msg;
        steering_wheel_msg.steer = last_steer_;
        steering_wheel_msg.header.stamp = time.now();
        //if(publish_th == true)
          pos_steering_wheel_publisher_->publish(steering_wheel_msg);
      }

    rclcpp::Publisher<epos_msgs::msg::ControlMode>::SharedPtr mode_publisher_;
    rclcpp::Publisher<epos_msgs::msg::Throttle>::SharedPtr pos_throttle_publisher_;
    rclcpp::Publisher<epos_msgs::msg::Brake>::SharedPtr pos_brake_publisher_;
    rclcpp::Publisher<epos_msgs::msg::SteeringWheel>::SharedPtr pos_steering_wheel_publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread processing_thread_; 

    int last_mode_;
    double last_throttle_;
    double last_brake_;
    double last_steer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeControl>());
  rclcpp::shutdown();
  return 0;
}