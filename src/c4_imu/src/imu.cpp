#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#define DEFAULT_FREQ 40
#define DEFAULT_BAUD_RATE 115200
#define COM_PORTBYID "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTXWA3VM-if00-port0"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class IMU :public rclcpp::Node
{
    public:
    IMU() : Node("imu")
    {
        _publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);

        imu_connection();
        publish_data();
    }

    private:
        void imu_connection(){
            //typedef uint32_t VN_ERROR_CODE;
            char* portName = COM_PORTBYID;
	        __uint32_t	baudRate = DEFAULT_BAUD_RATE;	
            __uint32_t 	freq = DEFAULT_FREQ;
	        bool isConnected = false;
            int portFd = -1;

            //openning port
            if(portFd = open(portName, O_RDWR | O_NOCTTY) < 0)
            {
                std::cout << " Failed Port Connection" << std::endl;
            }
            else
            {

            }
        }

        void publish_data(){
            auto message = std_msgs::msg::String();
            double yaw;
	        double pitch;
	        double roll;
            sscanf(message.data.c_str(), "$VNRRG,8,%+08.3f,%+08.3f ,%+08.3f", &yaw, &pitch, &roll);

            publisher_->publish(message);
        }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
      
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMU>());
  rclcpp::shutdown();
  return 0;
}