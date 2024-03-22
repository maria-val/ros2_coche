#include <memory>
#include <canlib.h>
#include "rclcpp/rclcpp.hpp"
#include "buscan_msgs/msg/can_raw.hpp"

typedef struct
{
  long id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
}raw_can_data_t;


class BusCan : public rclcpp::Node
{
    public:
        BusCan() : Node("buscan")
        {
            publisher_ = this->create_publisher<buscan_msgs::msg::CanRaw>("buscan", 100);

            initBuscanAndReadData();
        }

    private:
        void initBuscanAndReadData()
        {
            auto raw_message = buscan_msgs::msg::CanRaw();
            raw_can_data_t data;
            CanHandle h;
            bool init_buscan = false;
            int chan_count = 0;
            int channel;
            int read_data;

            canGetNumberOfChannels(&chan_count);

            if (chan_count <= 0 || chan_count > 64)
            {
                RCLCPP_ERROR(this->get_logger(), "Error getting channels");
                return;
            }

            channel = chan_count - 1;
            h = canOpenChannel(channel, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED);

            if (h < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Fail openning channel %d", channel);
                init_buscan = false;
            }
            else if(canSetBusParams(h, canBITRATE_500K, 4, 3, 1, 1, 0) != canOK ||
                    canSetBusOutputControl(h, canDRIVER_NORMAL) != canOK ||
                    canBusOn(h) != canOK)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize BUSCAN");
                init_buscan = false;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "BUSCAN initialized");
                init_buscan = true;
            }

            while(init_buscan)
            {
                read_data = canReadWait(h, &(data.id), &(data.msg), &(data.dlc), &(data.flag), &(data.time), -1);

                if(read_data != canOK && read_data != canERR_NOMSG)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error reading data");
                    init_buscan = false;
                }
                if(read_data == canOK)
                {
                    raw_message.id = data.id;
                    raw_message.time = data.time;
                    memcpy(&(raw_message.raw[0]), data.msg, sizeof(data.msg));
                    publisher_->publish(raw_message);
                }
            }
            canClose(h);
        }

        rclcpp::Publisher<buscan_msgs::msg::CanRaw>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BusCan>());
  rclcpp::shutdown();
  return 0;
}