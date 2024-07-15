#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "epos_msgs/msg/control_mode.hpp"
#include "epos_msgs/msg/throttle.hpp"
#include "epos_msgs/msg/brake.hpp"
#include "epos_msgs/msg/steering_wheel.hpp"
#include "buscan_msgs/msg/can_msg.hpp"
#include "c4_epos/EPOS.hpp"

using namespace std::chrono_literals;

class EposReader : public rclcpp::Node
{
  public:
    EposReader() : Node("epos_reader"), count_(0)
    {
      mode_subscription_ = this->create_subscription<epos_msgs::msg::ControlMode>("mode", 100, std::bind(&EposReader::ctrl_mode, this, std::placeholders::_1));
      throttle_subscription_ = this->create_subscription<epos_msgs::msg::Throttle>("ctrl_mode/throttle", 100, std::bind(&EposReader::set_throttle, this, std::placeholders::_1));
      break_subscription_ = this->create_subscription<epos_msgs::msg::Brake>("ctrl_mode/brake", 100, std::bind(&EposReader::set_brake, this, std::placeholders::_1));
      steer_subscription_ = this->create_subscription<epos_msgs::msg::SteeringWheel>("ctrl_mode/steer", 100, std::bind(&EposReader::set_steering_wheel, this, std::placeholders::_1));
      buscan_subscription_ = this->create_subscription<buscan_msgs::msg::CanMsg>("can_parser", 100, std::bind(&EposReader::buscan_data, this, std::placeholders::_1));
      
      throttle_publisher_ = this->create_publisher<epos_msgs::msg::Throttle>("epos_reader/throttle", 100);
      brake_publisher_ = this->create_publisher<epos_msgs::msg::Brake>("epos_reader/brake", 100);

      double eposvol_Kp = 586.0;
      double eposvol_Ki = 0.0;
      double eposvol_Kd = 0.0;
      eControlMode_t EPOS_initialization_mode;
      EPOS_initialization_mode = posicion;

      //initializing epos
      EPOS_brake = new EPOS(2, false);
      if(!EPOS_brake->GetStatus())
        EPOS_brake = new EPOS(2, true);

      EPOS_steering = new EPOS(1, false, eposvol_Kp, eposvol_Ki, eposvol_Kd, EPOS_initialization_mode);
      if(!EPOS_steering->GetStatus())
        EPOS_steering = new EPOS(1, true, eposvol_Kp, eposvol_Ki, eposvol_Kd, EPOS_initialization_mode);

      if(EPOS_brake->GetStatus() && EPOS_steering->GetStatus())
      {
        std::cerr << "\033[1;32m" << "Inicializadas EPOS ---> OK \033[0m" << std::endl;
      }
      else
      {
        std::cerr << "\033[1;31m" << "Fallo inicializando EPOS \033[0m" << std::endl;
      }

      //Once initialized, we deactivate automation
      EPOS_brake->SetDigitalOutput(false, 14);
      EPOS_brake->SetDigitalOutput(false, 15);

      timer_ = this->create_wall_timer(1s, std::bind(&EposReader::timer_callback, this));

      buscan_control_ = this->create_wall_timer(5s, std::bind(&EposReader::buscan_control_callback, this));
      steer_control_ = this->create_wall_timer(500ms, std::bind(&EposReader::steer_control_callback, this));
      throttle_control_ = this->create_wall_timer(500ms, std::bind(&EposReader::throttle_control_callback, this));
      brake_control_ = this->create_wall_timer(500ms, std::bind(&EposReader::brake_control_callback, this));
    }

  private:
    void timer_callback()
    {
      epos_msgs::msg::Throttle msg_throttle;
      epos_msgs::msg::Brake msg_brake;

      unsigned short value;
      EPOS_brake->GetAnalogInput(&value,1);
      msg_throttle.throttle = value;
      //RCLCPP_INFO(this->get_logger(), " ACelerador %d", value);

      msg_brake.brake = EPOS_brake->GetPos();
      //RCLCPP_INFO(this->get_logger(), "Freno %f", msg_brake.brake);

      rclcpp::Clock time;
      msg_throttle.header.stamp = time.now();
      throttle_publisher_->publish(msg_throttle);
      
      msg_brake.header.stamp = time.now();
      brake_publisher_->publish(msg_brake);
    }

    void buscan_control_callback()
    {
      rclcpp::Clock::SharedPtr my_clock = this->get_clock();
      rclcpp::Time current_time = my_clock->now();
      rclcpp::Duration time_bt_buscan_msgs = current_time - last_buscan_msg_;

      if(time_bt_buscan_msgs > rclcpp::Duration(2s))
      {
        RCLCPP_INFO(this->get_logger(), "Sin señal del buscan: desactivado modo automático");
        EPOS_brake->SetDigitalOutput(false, 14);
        EPOS_brake->SetDigitalOutput(false, 15);
      }
    }

    void throttle_control_callback()
    {
      /*if(gear_position < 3)
      {
        RCLCPP_INFO(this->get_logger(), "La marcha actual impide acelerar");
        return;
      }*/

      signed long brake_position = EPOS_brake->GetPos();
      std::cout << "Posicion freno " << brake_position << std::endl;
      /*if(brake_position < 0.0)
      {
        RCLCPP_INFO(this->get_logger(), "Freno activado, no acelerar");
        target_throttle = 0.0;
      }*/

      /*if(last_throttle_pos == set_throttle_pos)
        return;*/

      double target_throttle = set_throttle_pos/100;

      if (target_throttle < 0.0 )
        target_throttle = 0.0;
      else if (target_throttle > 1.0)
        target_throttle = 1.0;

      unsigned int throttle_tension = 150 + 1500.0 * target_throttle;
      EPOS_brake->SetAnalogOutput(throttle_tension);
    }

    void brake_control_callback()
    {
      if(!long_contr_act)
      {
        RCLCPP_INFO(this->get_logger(), "Modo automático no activado");
        return;
      }

      /*if(last_brake_pos == set_brake_pos)
        return;*/

      std::cout << "Freno " << set_brake_pos << std::endl;

      /*double target_brake = set_brake_pos/100;

      if (target_brake < 0.0 )
        target_brake = 0.0;
      else if (target_brake > 1.0)
        target_brake = 1.0;
        
      int brake_tension = -10000 + 40000 * target_brake;*/
    
      EPOS_brake->MoveAbsolute(set_brake_pos);
    }

    void steer_control_callback()
    {
      /*if(last_steering_wheel_pos == set_steering_wheel_pos)
        return;*/

      std::cout << "Volante " << set_steering_wheel_pos << std::endl;
      std::cout << "Posicion actual: " << steering_position << std::endl;

      double targetPositionSteering = round(set_steering_wheel_pos * 10);
      targetPositionSteering /= 10;

      double correction = targetPositionSteering - steering_position;

      if(fabs(correction) >= 0.1)
      {
          int total_count = correction * 900;
          RCLCPP_INFO(this->get_logger(), "Total steps per steering degree: %d", total_count);
          EPOS_steering->MoveRelative(total_count);
          last_steering_wheel_pos = set_steering_wheel_pos;
      }

      /*rclcpp::Clock::SharedPtr my_clock = this->get_clock();
      rclcpp::Time current_time = my_clock->now();
      rclcpp::Duration time_bt_steer_msgs = current_time - last_steer_msg_;

      if(time_bt_steer_msgs > rclcpp::Duration(2s))
      {
        RCLCPP_INFO(this->get_logger(), "Sin señal del volante: desactivado modo automático");
        EPOS_brake->SetDigitalOutput(false, 14);
        EPOS_brake->SetDigitalOutput(false, 15);
      }*/
    }

    void ctrl_mode(const epos_msgs::msg::ControlMode msg)
    {
      if(msg.mode == 1) 
      {
        //longitudinal and lateral controller
        longitudinal_control_activation();
        lateral_control_activation();
      }
      else if(msg.mode == 2) 
      {
        //longitudinal controller
        longitudinal_control_activation();
        EPOS_brake->SetDigitalOutput(false, 15);
      }
      else if(msg.mode == 3) 
      {
        //lateral controller
        lateral_control_activation();
        longitudinal_control_deactivation();

      }
      else if(msg.mode == 4)
      {
        //manual
        longitudinal_control_deactivation();
        EPOS_brake->SetDigitalOutput(false, 15);
      }
      else
      {
        EPOS_brake->SetDigitalOutput(false, 14);
        EPOS_brake->SetDigitalOutput(false, 15);
        RCLCPP_ERROR(this->get_logger(), "Error: modo desconocido");
      }
    }

    void set_throttle(const epos_msgs::msg::Throttle msg)
    {
      set_throttle_pos = msg.throttle;
      last_brake_msg_ = msg.header.stamp;
    }

    void set_brake(const epos_msgs::msg::Brake msg)
    {
      set_brake_pos = msg.brake;
      last_brake_msg_ = msg.header.stamp;
    }

    void set_steering_wheel(const epos_msgs::msg::SteeringWheel msg)
    {
      set_steering_wheel_pos = msg.steer;
      last_steer_msg_ = msg.header.stamp;
    }

    void buscan_data(const buscan_msgs::msg::CanMsg msg)
    {
      last_buscan_msg_ = msg.header.stamp;
      steering_position = msg.steer;
      brake_position = msg.brake;
      gear_position = msg.gear;
    }
    
    void longitudinal_control_activation()
    {
      EPOS_brake->SetAnalogOutput(150);
      EPOS_brake->MoveAbsolute(-20000);
      EPOS_brake->SetDigitalOutput(true, 14);
      long_contr_act = true;
    }
    void lateral_control_activation()
    {
      EPOS_brake->SetDigitalOutput(true, 15);
    }
    void longitudinal_control_deactivation()
    {
      EPOS_brake->SetAnalogOutput(150);
      EPOS_brake->MoveAbsolute(-20000);
      EPOS_brake->SetDigitalOutput(false, 14);
      long_contr_act = false;
    }
    
    EPOS* EPOS_brake;
    EPOS* EPOS_steering;

    float set_steering_wheel_pos = 0.0;
    float set_throttle_pos = 0.0;
    float set_brake_pos = 0.0;

    bool long_contr_act = false;

    float last_steering_wheel_pos = 0.0;
    float last_throttle_pos = 0.0;
    float last_brake_pos = 0.0;

    float steering_position = 0.0;
    float brake_position = 0.0;
    int gear_position = 0;

    rclcpp::Time last_throttle_msg_;
    rclcpp::Time last_brake_msg_;
    rclcpp::Time last_steer_msg_;
    rclcpp::Time last_buscan_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr buscan_control_;
    rclcpp::TimerBase::SharedPtr steer_control_;
    rclcpp::TimerBase::SharedPtr throttle_control_;
    rclcpp::TimerBase::SharedPtr brake_control_;

    rclcpp::Subscription<epos_msgs::msg::ControlMode>::SharedPtr mode_subscription_;
    rclcpp::Subscription<epos_msgs::msg::Throttle>::SharedPtr throttle_subscription_;
    rclcpp::Subscription<epos_msgs::msg::Brake>::SharedPtr break_subscription_;
    rclcpp::Subscription<epos_msgs::msg::SteeringWheel>::SharedPtr steer_subscription_;
    rclcpp::Subscription<buscan_msgs::msg::CanMsg>::SharedPtr buscan_subscription_;

    rclcpp::Publisher<epos_msgs::msg::Throttle>::SharedPtr throttle_publisher_;
    rclcpp::Publisher<epos_msgs::msg::Brake>::SharedPtr brake_publisher_;

    size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EposReader>());
  rclcpp::shutdown();
  return 0;
}