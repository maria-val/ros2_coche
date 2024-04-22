#include "rclcpp/rclcpp.hpp"
#include "c4_radar/radar_ars408.hpp"
#include "c4_radar/radar_srr208.hpp"

class RadarParser : public rclcpp::Node
{
  public:
    RadarParser() : Node("radar_parser")
    {
      ars408_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/ars408", 1000, std::bind(&RadarParser::parse_ars408_msg, this, std::placeholders::_1));
      srr208_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/srr208", 1000, std::bind(&RadarParser::parse_srr208_msg, this, std::placeholders::_1));
      parsed_ars408_publisher_ = this->create_publisher<radar_msgs::msg::RadarMsg408>("radar_parser/ars408", 1000);
      parsed_srr208_publisher_ = this->create_publisher<radar_msgs::msg::RadarMsg208>("radar_parser/srr208", 1000);
    }
  private:
    void parse_ars408_msg(const radar_msgs::msg::RadarRaw::SharedPtr msgIn)
    {
      RCLCPP_INFO(this->get_logger(), "Parsing T60 message");
      
      Radar_ARS408 radarARS(Radar_ARS408::ENABLE_60D);
      radarARS.borrarEstructura();
      radarARS.setRadarConfig(Radar_ARS408::ENABLE_60D);
      radarARS.m_radar_msg_pub = parsed_ars408_publisher_;
      radarARS.parse_radar_msg_408(msgIn);
    }

    void parse_srr208_msg(const radar_msgs::msg::RadarRaw::SharedPtr msgIn)
    {
      RCLCPP_INFO(this->get_logger(), "Parsing T61 or T62 message");

      Radar_SRR208 radarSRR;
      radarSRR.borrarEstructura();
      radarSRR.m_radar_SRR208_msg_pub = parsed_srr208_publisher_;
      radarSRR.parse_radar_msg(msgIn);
    }

    rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr ars408_subscriber_;
    rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr srr208_subscriber_;
    rclcpp::Publisher<radar_msgs::msg::RadarMsg408>::SharedPtr parsed_ars408_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarMsg208>::SharedPtr parsed_srr208_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarParser>());
  rclcpp::shutdown();
  return 0;
}