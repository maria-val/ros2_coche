#include <memory>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#include "rclcpp/rclcpp.hpp"
#include "nmea_msgs/msg/gpgga.hpp"

#define MSG_SZ 100

class TrimbleGps : public rclcpp::Node   //heredo método y funciones de la clase rclcpp::Node
{
  public:
    TrimbleGps()
        : Node("trimble_gps") //constructor que llama al constructor Node, es decir, el constructor crea un nodo en ROS2
    {
      publisher_ = this->create_publisher<nmea_msgs::msg::Gpgga>("gga", 10);

      gpgga_data();
    }

  private:
    void gpgga_data(){
      int port = 5018;
      int sockFd;
      struct sockaddr_in serv_addr;

      auto message = nmea_msgs::msg::Gpgga();
      char messagegps[MSG_SZ];
      char message_struct[MSG_SZ];
      int msgSize;

      char    c1,c2,c3,c4;
      int     i1,i2,i3,i4;
      double  d1,d2,d3,d4,d5,d6,d7;

      if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) < 0) //dir IPv4 y TCP
      {
        std::cout << " [OpenGPSDevice]: socket not create" << std::endl;
      }
      else
      {
        memset(&serv_addr, '0', sizeof(serv_addr));  //pone ceros en la variable serv_addr

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port); //se guarda el valor de port en big-endian (en orden)

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
            std::cout << " [OpenGPSDevice]: Connect " << std::endl;
          }
        }
      }

      while (1)
      {
        if ((msgSize = read(sockFd, messagegps, sizeof(messagegps) - 1)) > 0)
        {
          messagegps[msgSize] = '\0';
          if(messagegps != message_struct)
          {
            printf("%s\n", messagegps);
            sscanf(messagegps, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c,%lf,%d*%d", &d1,&d2,&c1,&d3,&c2,&i1,&i2,&d4,&d5,&c3,&d6,&c4,&d7,&i3,&i4);
            strcpy(message_struct, messagegps);

            //asignar cada valor que proporciona el GPS al mensaje NMEA correspondiente
            message.utc_seconds = d1;
            message.lat = d2;
            message.lon = d3;
            message.lat_dir = c1;
            message.lon_dir = c2;
            message.gps_qual = i1;
            message.num_sats = i2;
            message.hdop = d4;
            message.alt = d5;
            message.altitude_units = c3;
            message.undulation = d6;
            message.undulation_units = c4;
            message.diff_age = d7;
            message.station_id = i3;

            publisher_->publish(message);
          }
        }
      }
    }

  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrimbleGps>());
  rclcpp::shutdown();
  return 0;
}
