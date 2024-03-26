#include <memory> //for make_shared
#include <string.h> //for memset
#include <sys/socket.h> //for socket, connect
#include <unistd.h> //for read
#include <sstream>
#include <arpa/inet.h> //for inet_pton
#include <netinet/in.h> // for sockaddr_in struct, htons 
#include <gps_tools/conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nmea_msgs/msg/gpgga.hpp"

#define MSG_SZ 100

class TrimbleGps : public rclcpp::Node
{
  public:
    TrimbleGps() : Node("trimble_gps")
    {
      string_publisher_ = this->create_publisher<std_msgs::msg::String>("trimble/string", 10);
      navsat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("trimble/navsat_fix", 10);
      utm_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("trimble/position", 10);
      gga_publisher_ = this->create_publisher<nmea_msgs::msg::Gpgga>("trimble/gga", 10);
      getGpsData();
    }

  private:
    void getGpsData()
    {
      int port = 5018;
      int sock_fd;
      struct sockaddr_in serv_addr;
      bool is_gps_open = false;

      auto nmea_message = nmea_msgs::msg::Gpgga();
      auto std_message = std_msgs::msg::String();
      auto sensor_message = sensor_msgs::msg::NavSatFix();
      auto geom_message = geometry_msgs::msg::Point();

      char message_gps[MSG_SZ];
      int msg_size;

      std::string token;
      std::vector<std::string> tokens;

      int gps_qual, checksum, prev_checksum = -1;
      double lat, lon, alt, utm_n, utm_e;
      std::string utm_zone;

      //connecting to GPS
      if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) //dir IPv4 y TCP
      {
        RCLCPP_ERROR(this->get_logger(), "Socket not create");
        return;
      }
      else
      {
        memset(&serv_addr, '0', sizeof(serv_addr));  //sets 0s in the serv_addr variable

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port); //port is stored in big-endian (in order)

        if (inet_pton(AF_INET, "192.168.254.12", &serv_addr.sin_addr) <= 0) //It is required to change IP dir format to store it in serv_addr.sin_addr
        {
          RCLCPP_ERROR(this->get_logger(), "inet_pton error occurred");
          return;
        }
        else
        {
          if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed Connection");
            return;
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), "Connected");
            is_gps_open = true;
          }
        }
      }

      while (is_gps_open)
      {
        if ((msg_size = read(sock_fd, message_gps, sizeof(message_gps) - 1)) > 0)
        {
          std::stringstream ss(message_gps);

          //getting all the parameters of the GGA string
          while(getline(ss, token, ','))
          {
            tokens.push_back(token);
          }

          if(tokens.size() >= 10)
          {
            checksum = stoi(tokens[15]);

            //checking if the GGA string is different from the previous one
            if(checksum != prev_checksum)
            {
              prev_checksum = checksum;
              
              message_gps[msg_size] = '\0';
              RCLCPP_INFO(this->get_logger(), "%s", message_gps);
              //publishing the GGA string
              std_message.data = message_gps;
              string_publisher_->publish(std_message);

              //getting the required values for the NavSatFix message and assign them
              lat = stod(tokens[2]);
              lon = stod(tokens[4]);
              alt = stod(tokens[9]);
              gps_qual = stoi(tokens[6]);

              int lat_degrees = static_cast<int>(lat / 100);
              double lat_minutes = (lat - lat_degrees * 100.0) / 60.0;
              double lat_decimal = lat_degrees + lat_minutes;

              int lng_degrees = static_cast<int>(lon / 100);
              double lng_minutes = (lon - lng_degrees * 100.0) / 60.0;
              double lng_decimal = lng_degrees + lng_minutes;

              if (tokens[3] == "S")
              {
                lat_decimal *= -1.0;
              }

              if (tokens[5] == "W")
              {
                lng_decimal *= -1.0;
              }

              sensor_message.latitude = lat_decimal;
              sensor_message.longitude = lng_decimal;
              sensor_message.altitude = alt;

              switch (gps_qual)
              {
                case 0:
                  sensor_message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                  break;
                case 1:
                  sensor_message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                  break;
                case 2:
                  sensor_message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                  break;
                case 4:
                  sensor_message.status.status = 4; //RTK
                break;
                case 5:
                  sensor_message.status.status = 5; // Float RTK
                break;
                default:
                  sensor_message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                  break;
              }

              rclcpp::Clock time;
              sensor_message.header.stamp = time.now();
              navsat_publisher_->publish(sensor_message);

              //Turning decimal degrees of latitude and longitude into UTM coordinates
              gps_tools::LLtoUTM(lat_decimal, lng_decimal, utm_n, utm_e, utm_zone);
              geom_message.x = utm_e;
              geom_message.y = utm_n;
              geom_message.z = alt;
              utm_publisher_->publish(geom_message);
              
              //Assigning NMEA message values
              nmea_message.utc_seconds = stod(tokens[1]);
              nmea_message.lat = lat;
              nmea_message.lon = lon;
              nmea_message.lat_dir = tokens[3];
              nmea_message.lon_dir = tokens[5];
              nmea_message.gps_qual = gps_qual;
              nmea_message.num_sats = stoi(tokens[7]);
              nmea_message.hdop = stod(tokens[8]);
              nmea_message.alt = alt;
              nmea_message.altitude_units = tokens[10];
              nmea_message.undulation = stod(tokens[11]);
              nmea_message.undulation_units = tokens[12];
              nmea_message.diff_age = stod(tokens[13]);
              nmea_message.station_id = tokens[14];

              gga_publisher_->publish(nmea_message);
            }
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "GGA string does not contain enough fields to extract GPS values");
          }
        }
        //if there is any failure reading the string, close socket
        else
        {
          close(sock_fd);
          RCLCPP_ERROR(this->get_logger(), "Socket error");
          is_gps_open = false;
        }
      }
    }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr utm_publisher_;
  rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gga_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrimbleGps>());
  rclcpp::shutdown();
  return 0;
}