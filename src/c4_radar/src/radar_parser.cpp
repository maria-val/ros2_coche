#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/radar_raw.hpp"

class RadarParser : public rclcpp::Node
{
  public:
    RadarParser() : Node("radar_parser")
    {
      ars408_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/ars408", 1000, std::bind(&RadarParser::parse_radar_msg, this, std::placeholders::_1));
      srr208_subscriber_ = this->create_subscription<radar_msgs::msg::RadarRaw>("radar_reader/srr208", 1000, std::bind(&RadarParser::parse_radar_msg, this, std::placeholders::_1));
    }
  private:
  void parse_radar_msg(const radar_msgs::msg::RadarRaw msgIn)
  {

  }
  rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr ars408_subscriber_;
  rclcpp::Subscription<radar_msgs::msg::RadarRaw>::SharedPtr srr208_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarParser>());
  rclcpp::shutdown();
  return 0;
}