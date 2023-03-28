#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/gpgga.msg"

#define MSG_SZ 100
#define PCAP_TCP_SLL_HEADER 68
#define PCAP_TCP_HEADER 66

using namespace std::chrono_literals;

class TrimbleGps : public rclcpp::Node   //heredo método y funciones de la clase rclcpp::Node
{
public:
  TrimbleGps()
      : Node("trimble_gps") //constructor que llama al constructor Node, es decir, el constructor crea un nodo en ROS2
  {
    publisher_ = this->create_publisher<nmea_msgs::msg::Gpgga>("gga", 10);

  }

private:
  void timer_callback()
  {
    auto message = nmea_msgs::msg::Gpgga();
    // message.data = "Hello, world! " + std::to_string(count_++);
    int m_port = 5018;
    int sockFd;
    bool isGPSOpen = false;
    struct sockaddr_in serv_addr;
    char messagegps[100];
    int msgSize;

    if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      std::cout << " [OpenGPSDevice]: socket not create" << std::endl;
    }
    else
    {
      memset(&serv_addr, '0', sizeof(serv_addr));  //pone ceros en la variable serv_addr

      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(m_port); //se guarda el valor de m_port en big-endian (en orden)

      if (inet_pton(AF_INET, "192.168.254.12", &serv_addr.sin_addr) <= 0) //necesario convertir la direccion IP para almacenarla en serv_addr.sin_addr
      {
        std::cout << " [OpenGPSDevice]: inet_pton error occurred" << std::endl;
      }
      else
      {
        if (connect(sockFd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
          std::cout << " [OpenGPSDevice]: Failed Connection" << std::endl;
        }
        else
        {
          isGPSOpen = true;
          std::cout << " [OpenGPSDevice]: Connect " << std::endl;
        }
      }
    }

    while (1)
    {
      if ((msgSize = read(sockFd, messagegps, sizeof(messagegps) - 1)) > 0)
      {
        messagegps[msgSize] = '\0';
        if(messagegps != message.data)
        {
          printf("%s\n", messagegps);
          message.data = messagegps;
          publisher_->publish(message);
        }
      }
    }
  }

  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrimbleGps>());
  rclcpp::shutdown();
  return 0;
}
