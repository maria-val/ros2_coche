#include "rclcpp/rclcpp.hpp"
#include "c4_radar/radar_ars408.hpp"
#include "c4_radar/radar_srr208.hpp"

class RadarParser : public rclcpp::Node
{
  public:
    RadarParser() : Node("radar_parser")
    {
      auto header_ = declare_parameter<std::string>("header", "t60");
      auto topic_publisher_ = declare_parameter<std::string>("topic_publisher", "radar_parser/ars408");
      
      if(strcmp(header_.c_str(), "t60") == 0)
      {
        parsed_ars408_publisher_  = this->create_publisher<radar_msgs::msg::RadarMsg408>(topic_publisher_.c_str(), 1000);
        RCLCPP_INFO(this->get_logger(), "Parsing T60 message");

        Radar_ARS408 radarARS(Radar_ARS408::ENABLE_60D);
        radarARS.borrarEstructura();
        radarARS.setRadarConfig(Radar_ARS408::ENABLE_60D);
        radarARS.m_radar_msg_pub = parsed_ars408_publisher_;

        ars408_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/ars408", 1000, std::bind(&Radar_ARS408::parse_radar_msg_408, radarARS, std::placeholders::_1));

      }
      else if(strcmp(header_.c_str(), "t61") == 0)
      {
        parsed_t61_publisher_ = this->create_publisher<radar_msgs::msg::RadarMsg208>(topic_publisher_.c_str(), 1000);
        RCLCPP_INFO(this->get_logger(), "Parsing T61 message");

        Radar_SRR208 radarSRR;
        radarSRR.borrarEstructura();
        radarSRR.m_radar_SRR208_msg_pub = parsed_t61_publisher_;
        
        srr208_t61_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/t61", 1000, std::bind(&Radar_SRR208::parse_radar_msg, radarSRR, std::placeholders::_1));
      }
      else if(strcmp(header_.c_str(), "t62") == 0)
      {
        parsed_t62_publisher_ = this->create_publisher<radar_msgs::msg::RadarMsg208>(topic_publisher_.c_str(), 1000);
        RCLCPP_INFO(this->get_logger(), "Parsing T62 message");

        Radar_SRR208 radarSRR;
        radarSRR.borrarEstructura();
        radarSRR.m_radar_SRR208_msg_pub = parsed_t62_publisher_;
        
        srr208_t62_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/t62", 1000, std::bind(&Radar_SRR208::parse_radar_msg, radarSRR, std::placeholders::_1));
      }
    }

  private:
    rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr ars408_subscriber_;
    rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr srr208_t61_subscriber_;
    rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr srr208_t62_subscriber_;
    rclcpp::Publisher<radar_msgs::msg::RadarMsg408>::SharedPtr parsed_ars408_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarMsg208>::SharedPtr parsed_t61_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarMsg208>::SharedPtr parsed_t62_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarParser>());
  rclcpp::shutdown();
  return 0;
}