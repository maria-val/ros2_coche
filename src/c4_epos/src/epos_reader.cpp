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
    EposReader() : Node("epos_reader"), last_mode_msg_(0,0,RCL_ROS_TIME), last_buscan_msg_(0,0,RCL_ROS_TIME), last_throttle_msg_(0,0,RCL_ROS_TIME), last_brake_msg_(0,0,RCL_ROS_TIME), last_steer_msg_(0,0,RCL_ROS_TIME), count_(0)
    {
      mode_subscription_ = this->create_subscription<epos_msgs::msg::ControlMode>("ctrl_mode/mode", 100, std::bind(&EposReader::set_control_mode, this, std::placeholders::_1));
      throttle_subscription_ = this->create_subscription<epos_msgs::msg::Throttle>("ctrl_mode/throttle", 100, std::bind(&EposReader::set_throttle, this, std::placeholders::_1));
      break_subscription_ = this->create_subscription<epos_msgs::msg::Brake>("ctrl_mode/brake", 100, std::bind(&EposReader::set_brake, this, std::placeholders::_1));
      steer_subscription_ = this->create_subscription<epos_msgs::msg::SteeringWheel>("ctrl_mode/steer", 100, std::bind(&EposReader::set_steering_wheel, this, std::placeholders::_1));
      buscan_subscription_ = this->create_subscription<buscan_msgs::msg::CanMsg>("can_parser", 100, std::bind(&EposReader::buscan_data, this, std::placeholders::_1));
      
      throttle_publisher_ = this->create_publisher<epos_msgs::msg::Throttle>("epos_reader/manual_throttle_position", 100);
      brake_publisher_ = this->create_publisher<epos_msgs::msg::Brake>("epos_reader/brake_encoder_position", 100);
      steer_publisher_ = this->create_publisher<epos_msgs::msg::SteeringWheel>("epos_reader/steer_encoder_position", 100);

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
        std::cerr << "\033[1;32m" << "Inicializadas EPOS ---> OK \033[0m" << std::endl;
      
      else
        std::cerr << "\033[1;31m" << "Fallo inicializando EPOS \033[0m" << std::endl;

      //Once initialized, we deactivate automation
      longitudinal_control_deactivation();
      lateral_control_deactivation();

      timer_ = this->create_wall_timer(1s, std::bind(&EposReader::msg_control, this));
      general_control_ = this->create_wall_timer(500ms, std::bind(&EposReader::general_control_callback, this));      
    }

  private:
    void msg_control()
    {
      epos_msgs::msg::Throttle msg_throttle;
      epos_msgs::msg::Brake msg_brake;
      epos_msgs::msg::SteeringWheel msg_steer;

      rclcpp::Clock time;

      unsigned short value;
      EPOS_brake->GetAnalogInput(&value,1);
      msg_throttle.throttle = value;
      msg_throttle.header.stamp = time.now();
      throttle_publisher_->publish(msg_throttle);

      msg_brake.brake = EPOS_brake->GetPos();
      msg_brake.header.stamp = time.now();
      brake_publisher_->publish(msg_brake);

      msg_steer.steer = EPOS_steering->GetPos();
      msg_steer.header.stamp = time.now();
      steer_publisher_->publish(msg_steer);
    }

    void general_control_callback()
    {
      rclcpp::Clock::SharedPtr my_clock = this->get_clock();
      rclcpp::Time init_time = my_clock->now();

      mode_control();

      throttle_control();

      brake_control();

      steer_control();

      buscan_control();

      rclcpp::Time end_time = my_clock->now();
      rclcpp::Duration total_time = end_time - init_time;
      std::cout << "Tiempo total: " << total_time.seconds()<< std::endl;
    }

    void mode_control()
    {
      rclcpp::Clock::SharedPtr my_clock = this->get_clock();
      rclcpp::Time current_time = my_clock->now();
      rclcpp::Duration time_bt_mode_msgs = current_time - last_mode_msg_;

      if(time_bt_mode_msgs > rclcpp::Duration(2s))
      {
        RCLCPP_ERROR(this->get_logger(), "Sin señal del modo: modo automático desactivado");
        longitudinal_control_deactivation();
        lateral_control_deactivation();
        signal_error = true;
      }

      if(signal_error)
      set_mode_ = 4;

      if(set_mode_ != last_mode_)
      {
        if(set_mode_ == 1) //longitudinal and lateral controller
        {
          longitudinal_control_activation();
          lateral_control_activation();
        }
        else if(set_mode_ == 2) //longitudinal controller
        {
          longitudinal_control_activation();
          lateral_control_deactivation();
        }
        else if(set_mode_ == 3) //lateral controller
        {
          lateral_control_activation();
          longitudinal_control_deactivation();

        }
        else if(set_mode_ == 4) //manual
        {
          longitudinal_control_deactivation();
          lateral_control_deactivation();
        }
        else
        {
          longitudinal_control_deactivation();
          lateral_control_deactivation();
          RCLCPP_ERROR(this->get_logger(), "Error: modo desconocido");
        }

        last_mode_ = set_mode_;
      }

      std::cout << "Modo " << set_mode_ << std::endl;
    }

    void throttle_control()
    {
      if(set_mode_ == 1 || set_mode_ == 2)
      {
        rclcpp::Clock::SharedPtr my_clock = this->get_clock();
        rclcpp::Time current_time = my_clock->now();
        rclcpp::Duration time_bt_throttle_msgs = current_time - last_throttle_msg_;
    
        if(time_bt_throttle_msgs > rclcpp::Duration(2s))
        {
          RCLCPP_ERROR(this->get_logger(), "Sin señal del acelerador: modo automático desactivado");
          longitudinal_control_deactivation();
          lateral_control_deactivation();
          signal_error = true;
        }
        else
        {
          /*if(gear_position < 3)
          {
            RCLCPP_INFO(this->get_logger(), "La marcha actual impide acelerar");
            return;
          }*/

          std::cout << "Acelerador " << set_throttle_pos << std::endl;
          double target_throttle = set_throttle_pos/100;

          signed long brake_position = EPOS_brake->GetPos();
          std::cout << "Posicion freno " << brake_position << std::endl;

          brake_position += 5000;
          if(brake_position < 0)
          {
            RCLCPP_INFO(this->get_logger(), "Freno activado, no acelerar");
            target_throttle = 0.0;
          }

          //if(last_throttle_pos == set_throttle_pos)
            //return;

          if (target_throttle < 0.0 )
            target_throttle = 0.0;
          else if (target_throttle > 1.0)
            target_throttle = 1.0;

          unsigned int throttle_tension = 150 + 1500.0 * target_throttle;
          EPOS_brake->SetAnalogOutput(throttle_tension);
        }      
      }
    }

    void brake_control()
    {
      if(set_mode_ == 1 || set_mode_ == 2)
      {
        rclcpp::Clock::SharedPtr my_clock = this->get_clock();
        rclcpp::Time current_time = my_clock->now();
        rclcpp::Duration time_bt_brake_msgs = current_time - last_brake_msg_;

        if(time_bt_brake_msgs > rclcpp::Duration(2s))
        {
          RCLCPP_ERROR(this->get_logger(), "Sin señal del freno: modo automático desactivado");
          longitudinal_control_deactivation();
          lateral_control_deactivation();
          signal_error = true;
        }
        else
        {
          //if(last_brake_pos == set_brake_pos)
          //return;

          std::cout << "Freno " << set_brake_pos << std::endl;

          double target_brake = set_brake_pos/100;

          if (target_brake < 0.0 )
            target_brake = 0.0;
          else if (target_brake > 1.0)
            target_brake = 1.0;
            
          int brake_tension = - 30000 * target_brake;
        
          EPOS_brake->MoveAbsolute(brake_tension);
        }
      }
    }

    void steer_control()
    {
      if(set_mode_ == 1 || set_mode_ == 3)
      {
        rclcpp::Clock::SharedPtr my_clock = this->get_clock();
        rclcpp::Time current_time = my_clock->now();
        rclcpp::Duration time_bt_steer_msgs = current_time - last_steer_msg_;

        if(time_bt_steer_msgs > rclcpp::Duration(2s))
        {
          RCLCPP_ERROR(this->get_logger(), "Sin señal del volante: desactivado modo automático");
          longitudinal_control_deactivation();
          lateral_control_deactivation();
          signal_error = true;
        }
        else
        {
          std::cout << "Volante " << set_steering_wheel_pos << std::endl;
          std::cout << "Posicion actual: " << steering_position << std::endl;

          double targetPositionSteering = round(set_steering_wheel_pos * 10);
          targetPositionSteering /= 10;

          double correction = targetPositionSteering - steering_position;

          if(fabs(correction) >= 0.1)
          {
              int total_count = correction * 900;
              //RCLCPP_INFO(this->get_logger(), "Total steps per steering degree: %d", total_count);
              EPOS_steering->MoveRelative(total_count);
              last_steering_wheel_pos = set_steering_wheel_pos;
          }
        }
      }
    }

    void buscan_control()
    {
      rclcpp::Clock::SharedPtr my_clock = this->get_clock();
      rclcpp::Time current_time = my_clock->now();
      rclcpp::Duration time_bt_buscan_msgs = current_time - last_buscan_msg_;

      if(time_bt_buscan_msgs > rclcpp::Duration(2s))
      {
        RCLCPP_ERROR(this->get_logger(), "Sin señal del buscan: modo automático desactivado");
        longitudinal_control_deactivation();
        lateral_control_deactivation();
        signal_error = true;
      }
    }

    void set_control_mode(const epos_msgs::msg::ControlMode msg)
    {
      set_mode_ = msg.mode;
      last_mode_msg_ = msg.header.stamp;
    }

    void set_throttle(const epos_msgs::msg::Throttle msg)
    {
      set_throttle_pos = msg.throttle;
      last_throttle_msg_ = msg.header.stamp;
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
      EPOS_brake->MoveAbsolute(0);
      EPOS_brake->SetDigitalOutput(true, 14);
    }

    void lateral_control_activation()
    {
      EPOS_brake->SetDigitalOutput(true, 15);
    }
    
    void longitudinal_control_deactivation()
    {
      EPOS_brake->SetAnalogOutput(150);
      EPOS_brake->MoveAbsolute(0);
      EPOS_brake->SetDigitalOutput(false, 14);
    }

    void lateral_control_deactivation()
    {
      EPOS_brake->SetDigitalOutput(false, 15);
    }
    
    EPOS* EPOS_brake;
    EPOS* EPOS_steering;

    int set_mode_ = 0;
    float set_steering_wheel_pos = 0.0;
    float set_throttle_pos = 0.0;
    float set_brake_pos = 0.0;

    bool signal_error = false;

    int last_mode_ = 0;
    float last_steering_wheel_pos = 0.0;
    float last_throttle_pos = 0.0;
    float last_brake_pos = 0.0;

    float steering_position = 0.0;
    float brake_position = 0.0;
    int gear_position = 0;

    rclcpp::Time last_mode_msg_;
    rclcpp::Time last_throttle_msg_;
    rclcpp::Time last_brake_msg_;
    rclcpp::Time last_steer_msg_;
    rclcpp::Time last_buscan_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr general_control_;
    rclcpp::TimerBase::SharedPtr mode_control_;
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
    rclcpp::Publisher<epos_msgs::msg::SteeringWheel>::SharedPtr steer_publisher_;

    size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EposReader>());
  rclcpp::shutdown();
  return 0;
}