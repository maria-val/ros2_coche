#ifndef RADAR_ASR408_H
#define RADAR_ASR408_H

#include "c4_radar/UserDefinitions.h"
#include "rclcpp/rclcpp.hpp"
#include "radar_msgs/msg/radar_raw.hpp"
//#include "radar/ARS408_object.h"
#include "radar_msgs/msg/radar_msg408.hpp"
//#include <visualization_msgs/Marker.h>



//////////////////////////////////////////////////////////// ARS 408-21 ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class Radar_ARS408
{
  public:

    enum eConfigARS408
    {
      ENABLE_60B =0, // 60B On
      ENABLE_60C,    // 60B & 60C
      ENABLE_60BD,   // 60B & 60D
      ENABLE_60D     // 60B & 60C & 60D
    };
    Radar_ARS408();
    Radar_ARS408(enum eConfigARS408);
    
    Radar_ARS408_data_t m_dataRadar_ARS;

    std::shared_ptr<rclcpp::Publisher<radar_msgs::msg::RadarMsg408>> m_radar_msg_pub;

    int read_Radar(char* aux, Radar_ARS408_data_t* receivedData);
    void borrarEstructura();
    void borrarEstructura(Radar_ARS408_data_t* data);
    void setRadarConfig(eConfigARS408 config){m_radarConfig = config;};
    void parse_radar_msg_408(const radar_msgs::msg::RadarRaw msgIn);

  private:
    eConfigARS408 m_radarConfig;

    void read_Message_ID(/*in:*/ char* aux,
                        /*out:*/ int* msgID);

    //Radar_State (0x201)
    void read_201_Radar_State( /*in:*/ char* aux,
                              /*out:*/ bool*    NVMReadStatus,
                                       bool*    NVMwriteStatus,
                                       bool*    Voltage_Error,
                                       bool*    Temporary_Error,
                                       bool*    Temperature_Error,
                                       bool*    Interference,
                                       bool*    Persistent_Error,
                                       int*     MaxDistanceCfg,
                                       int*     SensorID,
                                       int*     SortIndex,
                                       int*     RadarPowerCfg,
                                       bool*    CtrlRelayCfg,
                                       int*     OutputTypeCfg,
                                       bool*    SendQualityCfg,
                                       bool*    SendExtInfoCfg,
                                       int*     MotionRxState,
                                       bool*    RCS_Threshold);


    //Object list status (0x60A)
    void read_60A_Object_0_Status_Message( /*in:*/  char*   aux,
                                           /*out:*/ int*    NumOfObjects,
                                                    int*    MeasCounter,
                                                    int*    InterfaceVersion);

    //Object general information (0x60B)
    void read_60B_Object_1_General_Message(  /*in:*/  char*     aux,
                                             /*out:*/ int*      Object_ID,
                                                      double*   Object_DistLong,
                                                      double*   Object_DistLat,
                                                      double*   Object_VrelLong,
                                                      t_DynProp* Object_DynProp,
                                                      double*   Object_VrelLat,
                                                      double*   Object_RCS);

    //Object quality information (0x60C)
    void read_60C_Object_2_Quality_Information( /*in:*/ char*    aux,
                                               /*out:*/ int*     Obj_ID, // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
                                                        double*  Obj_DistLong_rms, //Standard deviation of longitudinal distance
                                                        double*  Obj_VrelLong_rms, //Standard deviation of longitudinal relative velocity
                                                        double*  Obj_DistLat_rms, //Standard deviation of lateral distance
                                                        double*  Obj_VrelLat_rms, //Standard deviation of lateral relative velocity
                                                        double*  Obj_ArelLat_rms, //Standard deviation of lateral relative acceleration
                                                        double*  Obj_ArelLong_rms, //Standard deviation of longitudinal relative acceleration
                                                        double*  Obj_Orientation_rms, //Standard deviation of orientation angle
                                                        t_MeasState*  Obj_MeasState, //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
                                                        int* Obj_ProbOfExist); //Probability of existence (0:invalid, 1:<25%, 2:<50%, 3:<75%, 4:<90%, 5:<99%, 6:<99.9%, 7:<99.99%)

    //Object extended information (0x60D)
    void read_60D_Object_3_Extended_Message( /*in:*/  char*     aux,
                                             /*out:*/ int*      Obj_ID, // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
                                                      double*   Obj_ArelLong, //Relative acceleration in longitudinal direction
                                                      t_Class*  Obj_Class, //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
                                                      double*   Obj_ArelLat, //Relative acceleration in lateral direction
                                                      double*   Obj_OrientationAngle, //Orientation angle of the object
                                                      double*   Obj_Length, //Length of the tracked object
                                                      double*   Obj_Width); //Width of the tracked object)

    //Object collision detection warning (0x60E)
    void read_60E_Object_4_Warning_Message(  /*in:*/  char* aux,
                                             /*out:*/ int* nada);

    double auxInt2StdDeviation(int auxInt);
    double auxInt2StdDeviationDegrees(int auxInt);
};

#endif // RADAR_ASR408_H
