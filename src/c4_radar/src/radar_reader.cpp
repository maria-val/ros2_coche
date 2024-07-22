#include <memory>
#include <string.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/radar_raw.hpp"

class RadarReader : public rclcpp::Node
{
  public:
    RadarReader() : Node("radar_reader")
    {
      ars408_publisher_ = this->create_publisher<radar_msgs::msg::RadarRaw>("radar_reader/ars408", 1000);
      srr208_t61_publisher_ = this->create_publisher<radar_msgs::msg::RadarRaw>("radar_reader/t61", 1000);
      srr208_t62_publisher_ = this->create_publisher<radar_msgs::msg::RadarRaw>("radar_reader/t62", 1000);
      
      getRadarData();
    }

  private:
    void getRadarData()
    {
      radar_msgs::msg::RadarRaw radar_raw_msg;

      auto ip = declare_parameter<std::string>("ip", "192.168.254.254");
      char ip_array[20];
      strcpy(ip_array, ip.c_str());

      auto port = declare_parameter<int>("port", 2001);
      int sock_radar;
      struct sockaddr_in serv_addr;
      std::string header;
      bool is_radar_open = false;

      int count = 0;
      int bytes_read = 0;
      //cambiar read_temp de array a char
      char read_temp[23];
      char aux[23];
      memset(read_temp,'\0',23);
      memset(aux,'\0',23);
      
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
        if (inet_pton(AF_INET, ip_array, &serv_addr.sin_addr) <= 0)
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

      while(read_temp[0] != 't')
        count = read(sock_radar, &read_temp ,1 );

      if(read_temp[0] == 't')
      {
        aux[bytes_read] = read_temp[0]; 
        bytes_read++;
      }

      while(is_radar_open)
      {
        if((count = read(sock_radar, &read_temp, 1) == 1))
        {
          read_temp[23] = '\0';
          
          //ensure that the radar chain is passed completely
          if(read_temp[0] == 't')
          {
            aux[bytes_read] = '\0';
      
              //RCLCPP_INFO(this->get_logger(), "%s", read_temp);
              //printf("%s\n", aux );
              memcpy(&(radar_raw_msg.raw[0]), aux, sizeof(aux));

              //RCLCPP_INFO(this->get_logger(), "%s\n", radar_raw_msg.raw);
              /*for(int i=0; i < 23; i++)
                std::cout << "Cadena:" << radar_raw_msg.raw[i] << std::endl;*/
              rclcpp::Clock time;
              radar_raw_msg.header.stamp = time.now();

              //choose the topic in which the message has to be published
              if(strncmp(aux, "t60", 3) == 0)
                ars408_publisher_->publish(radar_raw_msg);
      
              else if(strncmp(aux, "t61", 3) == 0)
                srr208_t61_publisher_->publish(radar_raw_msg);
              
              else if(strncmp(aux, "t62", 3) == 0)
                srr208_t62_publisher_->publish(radar_raw_msg);
            
              else
                RCLCPP_INFO(this->get_logger(), "Message from another radar: %s", aux);

               memset(aux,'\0',23);
               bytes_read = 0;
          }
          aux[bytes_read] = read_temp[0]; 
          bytes_read++;
        }
      }
    }
    rclcpp::Publisher<radar_msgs::msg::RadarRaw>::SharedPtr ars408_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarRaw>::SharedPtr srr208_t61_publisher_;
    rclcpp::Publisher<radar_msgs::msg::RadarRaw>::SharedPtr srr208_t62_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarReader>());
  rclcpp::shutdown();
  return 0;
}