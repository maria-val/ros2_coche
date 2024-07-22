#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* strtoul */

#include "c4_radar/radar_srr208.hpp"

Radar_SRR208::Radar_SRR208()
{}

int Radar_SRR208::read_Radar(char* aux, Radar_SRR_data_t* receivedData)
{
    int retValue = 1;
    receivedData->status.completed = false;

    int msgID;
    read_Message_ID(aux, &msgID);

   // variables temporales    
   int 		Track_ID =0;
   double 	Track_LongDispl=0;
   int 		Track_Index=0;
   double 	Track_LatDispl=0;
   double 	Track_VrelLong=0;
   int 		Track_Roll_Count=0;
   double 	Track_VrelLat=0;
   
   double 	Track_RCSValue =0;
   double 	Track_Lifetime=0;
   int 		Track_Index2 =0;
   int 		Track_Roll_Count2 =0;

    //The track list output consists of three messages: CAN1_VersionID (0x700),
    //CAN1_Track_Status (0x60B), CAN1_Track_1 (0x60C) and CAN1_Track_2 (0x60D). The
    //message CAN1_Track_Status contains general information about the track list itself, for
    //example the number of tracks it currently contains. The messages CAN_Track1 and
    //CAN1_Track2 contain track specific attributes like position and relative velocity. While the
    //message CAN1_Track_Status is sent only once during one transmission cycle,
    //CAN1_Track1 and CAN1_Track2 are sent multiple times as message pairs for each track
    //in the track list. The maximum number of tracks in the track list cannot exceed 25, therefore
    //(in contrast to the cluster lists) it is possible to send an updated track list each radar cycle.
    //One radar cycle corresponds to approx. 33 milliseconds.

    switch (msgID)
    {
    case SHORT_CAN1_Track_Status_ID1: // 1563 (0x61B) CAN1_Track_Status (ID 1)
    case SHORT_CAN1_Track_Status_ID2: // 1579 (0x62B) CAN1_Track_Status (ID 2)
        borrarEstructura(receivedData);
        if (msgID == 1563) receivedData->status.RadarID = 1;
        else if (msgID == 1579) receivedData->status.RadarID = 2;
        
	      read_60B_CAN1_Track_Status_Message(aux, &(receivedData->status.NumOfTracks), &(receivedData->status.TrackSt_RollCount));
        break;

    case SHORT_CAN1_Track_1_ID1: // 1564 (0x61C) CAN1_Track_1 (ID 1)
    case SHORT_CAN1_Track_1_ID2: // 1580 (0x62C) CAN1_Track_1 (ID 2)
        if (msgID == 1564) receivedData->status.RadarID = 1;
        else if (msgID == 1580) receivedData->status.RadarID = 2;

        read_60C_CAN1_Track1_Message(aux,   &Track_ID,
                                            &Track_LongDispl,
                                            &Track_Index,
                                            &Track_LatDispl,
                                            &Track_VrelLong,
                                            &Track_Roll_Count,
                                            &Track_VrelLat);
	
	receivedData->status.receivedNumOfTracks1 = Track_Index;	     
		 
	receivedData->objects[Track_Index].Track_ID =Track_ID;
        receivedData->objects[Track_Index].Track_LongDispl = Track_LongDispl;
        receivedData->objects[Track_Index].Track_Index = Track_Index;
        receivedData->objects[Track_Index].Track_LatDispl=Track_LatDispl;
        receivedData->objects[Track_Index].Track_VrelLong=Track_VrelLong;
        receivedData->objects[Track_Index].Track_Roll_Count=Track_Roll_Count;
        receivedData->objects[Track_Index].Track_VrelLat=Track_VrelLat;		
        break;

    case SHORT_CAN1_Track_2_ID1: // 1565 (0x61D) CAN1_Track_2 (ID 1)
    case SHORT_CAN1_Track_2_ID2: // 1581 (0x62D) CAN1_Track_2 (ID 2)
        if (msgID == 1565) receivedData->status.RadarID = 1;
        else if (msgID == 1581) receivedData->status.RadarID = 2;

        read_60D_CAN1_Track2_Message(aux,   &Track_RCSValue,
                                            &Track_Lifetime,
                                            &Track_Index2,
                                            &Track_Roll_Count2);
        
	receivedData->status.receivedNumOfTracks2 = Track_Index2;
        receivedData->objects[Track_Index2].Track_RCSValue = Track_RCSValue;
        receivedData->objects[Track_Index2].Track_Lifetime = Track_Lifetime;
        receivedData->objects[Track_Index2].Track_Index2 = Track_Index2;
        receivedData->objects[Track_Index2].Track_Roll_Count2 = Track_Roll_Count2;

        if(receivedData->status.receivedNumOfTracks2 == (receivedData->status.NumOfTracks -1))
        {
            receivedData->status.completed = true;
        }
        else
        {
            receivedData->status.completed = false;
        }
        break;

    default:
        retValue = -1;
        //printf("SSR08 ret -1\n");
        break;
    }
    return retValue;
}

void Radar_SRR208::read_Message_ID(/*in:*/ char* aux,
                                  /*out:*/ int* msgID)
{
    char fourByte[5];
    fourByte[4]='\0';

    fourByte[0] = '0';
    fourByte[1] = aux[1];
    fourByte[2] = aux[2];
    fourByte[3] = aux[3];
    *msgID = strtoul(fourByte,NULL,16);
    *msgID &= 0x0FFF;
}

void Radar_SRR208::read_60B_CAN1_Track_Status_Message(/*in:*/ char* aux,
                                                     /*out:*/ int* NumOfTracks,
                                                              int* TrackSt_RollCount)
{
    char twoByte[3];
    twoByte[2]='\0';

    twoByte[0] = aux[5];
    twoByte[1] = aux[6];
    *NumOfTracks = strtoul(twoByte,NULL,16);

    twoByte[0] = aux[7];
    twoByte[1] = aux[8];
    *TrackSt_RollCount = strtoul(twoByte,NULL,16) & 0x03;
}

void Radar_SRR208::read_60C_CAN1_Track1_Message(/*in:*/ char* aux,
                                               /*out:*/ int* Track_ID,
                                                        double* Track_LongDispl,
                                                        int* Track_Index,
                                                        double* Track_LatDispl,
                                                        double* Track_VrelLong,
                                                        int* Track_Roll_Count,
                                                        double* Track_VrelLat)
{
    char oneByte[2];
    oneByte[1]='\0';
    char twoByte[3];
    twoByte[2]='\0';
    char fourByte[5];
    fourByte[4]='\0';

    int int_Track_LongDispl;
    int int_Track_LatDispl;
    int int_Track_VrelLong;
    int int_Track_VrelLat;

    fourByte[0] = aux[5];
    fourByte[1] = aux[6];
    fourByte[2] = aux[7];
    fourByte[3] = aux[8];
    *Track_ID = strtoul(fourByte,NULL,16);

    fourByte[0] = aux[9];
    fourByte[1] = aux[10];
    fourByte[2] = aux[11];
    fourByte[3] = aux[12];
    int_Track_LongDispl = strtoul(fourByte,NULL,16);
    int_Track_LongDispl = int_Track_LongDispl >> 5;
    int_Track_LongDispl &= 0x01FF;
    *Track_LongDispl = ((double)int_Track_LongDispl)*0.1;

    twoByte[0] = aux[11];
    twoByte[1] = aux[12];
    *Track_Index = strtoul(twoByte,NULL,16);
    *Track_Index &= 0x1F;

    fourByte[0] = aux[13];
    fourByte[1] = aux[14];
    fourByte[2] = aux[15];
    fourByte[3] = aux[16];
    int_Track_LatDispl = strtoul(fourByte, NULL, 16);
    int_Track_LatDispl = int_Track_LatDispl >> 6;
    int_Track_LatDispl &= 0x03FF;
    *Track_LatDispl = ((double)int_Track_LatDispl*0.1)-51.1;

    fourByte[0] = aux[15];
    fourByte[1] = aux[16];
    fourByte[2] = aux[17];
    fourByte[3] = aux[18];
    int_Track_VrelLong = strtoul(fourByte, NULL, 16);
    int_Track_VrelLong &= 0x3FFC;
    int_Track_VrelLong = int_Track_VrelLong >> 2;
    //*Track_VrelLong = ((double)int_Track_VrelLong*0.0625)-128.0;
    *Track_VrelLong = ((double)int_Track_VrelLong*0.02)-32.0;

    oneByte[0] = aux[18];
    *Track_Roll_Count = strtoul(oneByte, NULL, 16);
    *Track_Roll_Count &= 0x3;

    twoByte[0] = aux[19];
    twoByte[1] = aux[20];
    int_Track_VrelLat = strtoul(twoByte, NULL, 16);
    *Track_VrelLat = ((double)int_Track_VrelLat*0.25)-32.0;
}

void Radar_SRR208::read_60D_CAN1_Track2_Message(/*in:*/ char* aux,
                                               /*out:*/ double* Track_RCSValue,
                                                        double* Track_Lifetime,
                                                        int* Track_Index2,
                                                        int* Track_Roll_Count2)
{
    char oneByte[2];
    oneByte[1]='\0';
    char twoByte[3];
    twoByte[2]='\0';
    char fourByte[5];
    fourByte[4]='\0';

    int int_Track_RCSValue;
    int int_Track_Lifetime;

    twoByte[0] = aux[5];
    twoByte[1] = aux[6];
    int_Track_RCSValue  = strtoul(twoByte, NULL, 16);
    //*Track_RCSValue = ((double)int_Track_RCSValue*0.5)-64.0;
    *Track_RCSValue = ((double)int_Track_RCSValue*0.5)-50.0;

    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    int_Track_Lifetime = strtoul(fourByte, NULL, 16);
    //*Track_Lifetime = int_Track_Lifetime;
    *Track_Lifetime = (double)int_Track_Lifetime*0.1;

    twoByte[0] = aux[11];
    twoByte[1] = aux[12];
    *Track_Index2 = strtoul(twoByte, NULL, 16);
    *Track_Index2 &= 0x1F;

    oneByte[0] = aux[14];
    *Track_Roll_Count2  = strtoul(oneByte, NULL, 16);
    *Track_Roll_Count2  &= 0x3;
}

void Radar_SRR208::borrarEstructura(Radar_SRR_data_t* data)
{
    data->status.RadarID               = 0;
    data->status.NumOfTracks           = 0;
    data->status.TrackSt_RollCount     = 0;
    data->status.receivedNumOfTracks1  = 0;
    data->status.receivedNumOfTracks2  = 0;
    data->status.completed             = false;

    for (int i = 0; i<MAX_OBJECTS_208; i++)
    {
        data->objects[i].Track_ID          = 0;
        data->objects[i].Track_LongDispl   = 0;
        data->objects[i].Track_Index       = 0;
        data->objects[i].Track_LatDispl    = 0;
        data->objects[i].Track_VrelLong    = 0;
        data->objects[i].Track_Roll_Count  = 0;
        data->objects[i].Track_VrelLat     = 0;

        data->objects[i].Track_RCSValue    = 0;
        data->objects[i].Track_Lifetime    = 0;
        data->objects[i].Track_Index2      = 0;
        data->objects[i].Track_Roll_Count2 = 0;
    }
}

void Radar_SRR208::borrarEstructura()
{
    m_dataRadar_SSR.status.RadarID               = 0;
    m_dataRadar_SSR.status.NumOfTracks           = 0;
    m_dataRadar_SSR.status.TrackSt_RollCount     = 0;
    m_dataRadar_SSR.status.receivedNumOfTracks1  = 0;
    m_dataRadar_SSR.status.receivedNumOfTracks2  = 0;
    m_dataRadar_SSR.status.completed             = false;

    for (int i = 0; i<MAX_OBJECTS_208; i++)
    {
        m_dataRadar_SSR.objects[i].Track_ID          = 0;
        m_dataRadar_SSR.objects[i].Track_LongDispl   = 0;
        m_dataRadar_SSR.objects[i].Track_Index       = 0;
        m_dataRadar_SSR.objects[i].Track_LatDispl    = 0;
        m_dataRadar_SSR.objects[i].Track_VrelLong    = 0;
        m_dataRadar_SSR.objects[i].Track_Roll_Count  = 0;
        m_dataRadar_SSR.objects[i].Track_VrelLat     = 0;

        m_dataRadar_SSR.objects[i].Track_RCSValue    = 0;
        m_dataRadar_SSR.objects[i].Track_Lifetime    = 0;
        m_dataRadar_SSR.objects[i].Track_Index2      = 0;
        m_dataRadar_SSR.objects[i].Track_Roll_Count2 = 0;
    }
}

void Radar_SRR208::parse_radar_msg(const radar_msgs::msg::RadarRaw & msgIn)
{
    radar_msgs::msg::RadarMsg208 msg;
    char aux[23];

    memcpy(aux, (const char*) (&(msgIn.raw[0])), 22);
    aux[23]='\0';

    RCLCPP_INFO(rclcpp::get_logger("radar_parser"),"I heard: [%d:%d  %s]", msgIn.header.stamp.sec, msgIn.header.stamp.nanosec, aux);

    read_Radar(aux, &m_dataRadar_SSR);
    if(m_dataRadar_SSR.status.completed)
    {
        m_dataRadar_SSR.status.completed = false;
        RCLCPP_INFO(rclcpp::get_logger("radar"), " SRR COMPLETADO NumOfTracks = %d", m_dataRadar_SSR.status.NumOfTracks);
        msg.header.stamp = msgIn.header.stamp;

        copyRadar_SRR208_data2msg(m_dataRadar_SSR, &msg);
        m_radar_SRR208_msg_pub->publish(msg);
    }
}

void Radar_SRR208::copyRadar_SRR208_data2msg(Radar_SRR_data_t data, radar_msgs::msg::RadarMsg208 *msg)
{
    msg->radar_id = data.status.RadarID;
    msg->num_of_tracks = data.status.NumOfTracks;
    msg->track_st_roll_count = data.status.TrackSt_RollCount;
    msg->received_num_of_tracks1 = data.status.receivedNumOfTracks1;
    msg->received_num_of_tracks2 = data.status.receivedNumOfTracks2;
    msg->completed = data.status.completed;

    for (int i=0; i<25;i++)
    {
        msg->objects[i].track_id = data.objects[i].Track_ID;
        msg->objects[i].track_long_displ = data.objects[i].Track_LongDispl;
        msg->objects[i].track_index = data.objects[i].Track_Index;
        msg->objects[i].track_lat_displ = data.objects[i].Track_LatDispl;
        msg->objects[i].track_vrel_long = data.objects[i].Track_VrelLong;
        msg->objects[i].track_roll_count = data.objects[i].Track_Roll_Count;
        msg->objects[i].track_vrel_lat = data.objects[i].Track_VrelLat;
        msg->objects[i].track_rcs_value = data.objects[i].Track_RCSValue;
        msg->objects[i].track_lifetime = data.objects[i].Track_Lifetime;
        msg->objects[i].track_index2 = data.objects[i].Track_Index2;
        msg->objects[i].track_roll_count2 = data.objects[i].Track_Roll_Count2;
    }
}

