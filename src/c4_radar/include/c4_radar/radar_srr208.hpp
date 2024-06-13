#ifndef RADAR_SRR208_H
#define RADAR_SRR208_H


#include "c4_radar/UserDefinitions.h"
#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/String.h"
#include "radar_msgs/msg/radar_raw.hpp"
#include "radar_msgs/msg/radar_msg208.hpp"

//////////////////////////////////////////////////////////// SRR-208-21 ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define SHORT_CAN1_Track_Status_ID0  1547 // (0x60B) CAN1_Track_Status (ID 0) from Manual not tested
//#define SHORT_CAN1_Track_1_ID0       1548 // (0x60C) CAN1_Track_1 (ID 0) from Manual not tested
//#define SHORT_CAN1_Track_2_ID0       1549 // (0x61D) CAN1_Track_2 (ID 0) from Manual not tested
#define SHORT_CAN1_Track_Status_ID2  1579 // (0x62B) CAN1_Track_Status (ID 2)
#define SHORT_CAN1_Track_1_ID2       1580 // (0x62C) CAN1_Track_1 (ID 2)
#define SHORT_CAN1_Track_2_ID2       1581 // (0x62D) CAN1_Track_2 (ID 2)
#define SHORT_CAN1_Track_Status_ID1  1563 // (0x61B) CAN1_Track_Status (ID 1)
#define SHORT_CAN1_Track_1_ID1       1564 // (0x61C) CAN1_Track_1 (ID 1)
#define SHORT_CAN1_Track_2_ID1       1565 // (0x61D) CAN1_Track_2 (ID 1)

class Radar_SRR208
{
public:
    Radar_SRR208();

    Radar_SRR_data_t m_dataRadar_SSR;

    std::shared_ptr<rclcpp::Publisher<radar_msgs::msg::RadarMsg208>> m_radar_SRR208_msg_pub;

    int read_Radar(char* aux, Radar_SRR_data_t* receivedData);
    void borrarEstructura(Radar_SRR_data_t* data);
    void borrarEstructura();
    void parse_radar_msg(const radar_msgs::msg::RadarRaw & msgIn);
    void copyRadar_SRR208_data2msg(Radar_SRR_data_t data, radar_msgs::msg::RadarMsg208 *msg);
    

private:
    void read_Message_ID(/*in:*/ char* aux, /*out:*/ int* msgID);

    void read_60B_CAN1_Track_Status_Message(/*in:*/ char* aux,
                                           /*out:*/ int* NumOfTracks,
                                                    int* TrackSt_RollCount);

    void read_60C_CAN1_Track1_Message(/*in:*/ char* aux,
                                     /*out:*/ int* Track_ID,
                                              double* Track_LongDispl,
                                              int* Track_Index,
                                              double* Track_LatDispl,
                                              double* Track_VrelLong,
                                              int* Track_Roll_Count,
                                              double* Track_VrelLat);

    void read_60D_CAN1_Track2_Message(/*in:*/ char* aux,
                                     /*out:*/ double* Track_RCSValue,
                                              double* Track_Lifetime,
                                              int* Track_Index2,
                                              int* Track_Roll_Count2);
};

#endif
