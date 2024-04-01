#include <memory>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/radar_raw.hpp"

class RadarReader : public rclcpp::Node
{
  public:
    RadarReader() : Node("radar_reader")
    {
      ars408_publisher_ = this->create_publisher<radar_msgs::msg::RadarRaw>("radar_reader/ars408", 1000);
      srr208_publisher_ = this->create_publisher<radar_msgs::msg::RadarRaw>("radar_reader/srr208", 1000);
      getRadarData();
    }

  private:
    void getRadarData()
    {
      auto radar_raw_msg = radar_msgs::msg::RadarRaw();

      char ip[20] = "192.168.254.254";
      int port = 2001;
      int sock_radar;
      struct sockaddr_in serv_addr;
      std::string header;
      bool is_radar_open = false;
      
      if((sock_radar = socket(AF_INET, SOCK_STREAM, 0)) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Socket not create");
        return;
      }
      else
      {
        memset(&serv_addr, '0', sizeof(serv_addr));

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) <= 0)
        {
          RCLCPP_ERROR(this->get_logger(), "inet_pton error occurred");
          return;
        }
        else
        {
          if (connect(sock_radar, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed Connection");
            return;
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Connected");
            is_radar_open = true;
          }
        }
      }

      if(is_radar_open)
      {
        int bytes_read;
        char read_temp[23];
        memset(read_temp,'\0',23);

        if((bytes_read = read(sock_radar, &read_temp, sizeof(read_temp))) > 0)
        {
          RCLCPP_INFO(this->get_logger(), "%s", read_temp);

          if(strncmp(read_temp, "t60", 3) == 0)
          {
            memcpy(&(radar_raw_msg.raw[0]), read_temp, sizeof(read_temp));
            rclcpp::Clock time;
            radar_raw_msg.header.stamp = time.now();
            ars408_publisher_->publish(radar_raw_msg);
          }
          else if(strncmp(read_temp, "t61", 3) == 0 || strncmp(read_temp, "t62", 3) == 0)
          {
            memcpy(&(radar_raw_msg.raw[0]), read_temp, sizeof(read_temp));
            rclcpp::Clock time;
            radar_raw_msg.header.stamp = time.now();
            srr208_publisher_->publish(radar_raw_msg);
          }
          else
            RCLCPP_INFO(this->get_logger(), "Message from another radar");
        }
        else
        {
          close(sock_radar);
          is_radar_open = false;
          RCLCPP_ERROR(this->get_logger(), "Socket error");
        }      
      }
    }
    rclcpp::Publisher<radar_msgs::msg::RadarRaw>::SharedPtr ars408_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarRaw>::SharedPtr srr208_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarReader>());
  rclcpp::shutdown();
  return 0;
}