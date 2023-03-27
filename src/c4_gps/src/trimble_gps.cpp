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
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TrimbleGps::timer_callback, this));
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
    // gps->m_gpsParser = new GPS_parser(gps->m_gpsMessageType);
    //int returnValue = 0;

    /*GPS_parser::GPS_dataRMC_t gpsdataRMC;
    GPS_parser::GPS_dataGGA_t gpsdataGGA; */

    if ((sockFd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      std::cout << " [OpenGPSDevice]: not create socket" << std::endl;
    }
    else
    {
      memset(&serv_addr, '0', sizeof(serv_addr));

      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(m_port);

      if (inet_pton(AF_INET, "192.168.254.12", &serv_addr.sin_addr) <= 0)
      {
        std::cout << " [OpenGPSDevice]: inet_pton error occurred" << std::endl;
      }
      else
      {
        if (connect(sockFd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
          std::cout << " [OpenGPSDevice]: Connect Failed " << std::endl;
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
        printf("%s\n", messagegps);
        message.data = messagegps;
        publisher_->publish(message);
        // call process data
        /* if(gps->m_gpsParser->read_GPS(message)> 0)
         {
             switch(gps->m_gpsMessageType)
             {
             case GPS_parser::RMC:
                 gps->m_gpsParser->get_GPSstruct(&gpsdataRMC);
                 //gps->m_gpsRMCSync.Enqueue(&gpsdataRMC);
                 break;
             case GPS_parser::GGA:
                 gps->m_gpsParser->get_GPSstruct(&gpsdataGGA);
                 pthread_mutex_lock(&(gps->m_gps_GGA_data_mutex));
                 gps->m_gps_GGA_data = gpsdataGGA;
                 pthread_mutex_unlock(&(gps->m_gps_GGA_data_mutex));
                 //sem_post(gps->m_HLC_sem);
                 if(gpsFile != NULL)
                     fprintf(gpsFile, "%.2lf %.3lf %.3lf %.3lf %d\n",
                             gpsdataGGA.UTC_time,
                             gpsdataGGA.writingTimeMS,
                             gpsdataGGA.Easting,
                             gpsdataGGA.Northing,
                             gpsdataGGA.Quality);
                 break;
             default:
                 break;
             }
         }
 */
      }
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
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
