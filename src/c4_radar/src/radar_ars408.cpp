#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* strtoul */

#include "c4_radar/radar_ars408.hpp"

Radar_ARS408::Radar_ARS408()
{
    m_radarConfig =ENABLE_60D;
}

Radar_ARS408::Radar_ARS408(enum eConfigARS408 configParam)
{
    m_radarConfig = configParam;
}

int Radar_ARS408::read_Radar(char aux[23], Radar_ARS408_data_t* receivedData)
{
    int retValue = 1;

    int msgID;

    //Obj Data ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    int     Obj_ID; // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
    //0x60B
    double  Obj_LongDispl;   //Longitudinal displacement
    double  Obj_LatDispl;    //Lateral displacement
    double  Obj_VrelLong;    //Relative longitudinal speed
    t_DynProp   Object_DynProp; //Dynamic property of the object indicating if the object is moving or stationary (this value can only be determined correctly if the speed and yaw rate is given correctly)
    double  Obj_LatSpeed;    //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)
    double  Obj_RCSValue;    //Radar cross section (RCS) is the measure of the reflective strength of a target or in other words it is the measure of power scattered in a given spatial direction when a target is illuminated by an incident wave.
    //0x60C
    double  Obj_DistLong_rms; //Standard deviation of longitudinal distance
    double  Obj_VrelLong_rms; //Standard deviation of longitudinal relative velocity
    double  Obj_DistLat_rms; //Standard deviation of lateral distance
    double  Obj_VrelLat_rms; //Standard deviation of lateral relative velocity
    double  Obj_ArelLat_rms; //Standard deviation of lateral relative acceleration
    double  Obj_ArelLong_rms; //Standard deviation of longitudinal relative acceleration
    double  Obj_Orientation_rms; //Standard deviation of orientation angle
    t_MeasState  Obj_MeasState; //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
    int     Obj_ProbOfExist; //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%
    //0x60D
    double  Obj_ArelLong; //Relative acceleration in longitudinal direction
    t_Class Obj_Class; //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
    double  Obj_ArelLat; //Relative acceleration in lateral direction
    double  Obj_OrientationAngle; //Orientation angle of the object
    double  Obj_Length; //Length of the tracked object
    double  Obj_Width; //Width of the tracked object
    //Obj Data ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    read_Message_ID(aux, &msgID);

    // printf("Msg ARS408 read_Message_I : %s\n",aux);

    int basura;

    //Object List
    //If the quality information, extended information and/or warning state is sent, first all
    //messages of type Object_1_General (0x60B) are sent and afterwards all messages of type
    //Object_2_Quality (0x60C), afterwards of type Object_3_Extended (0x60D) and/or
    //afterwards of type Object_4_Warning (0x60E).
    switch (msgID)
    {
    case 513: // (0x201) Radar State
        read_201_Radar_State(aux, &(receivedData->radarState.NVMReadStatus),
                             &(receivedData->radarState.NVMwriteStatus),
                             &(receivedData->radarState.Voltage_Error),
                             &(receivedData->radarState.Temporary_Error),
                             &(receivedData->radarState.Temperature_Error),
                             &(receivedData->radarState.Interference),
                             &(receivedData->radarState.Persistent_Error),
                             &(receivedData->radarState.MaxDistanceCfg),
                             &(receivedData->radarState.SensorID),
                             &(receivedData->radarState.SortIndex),
                             &(receivedData->radarState.RadarPowerCfg),
                             &(receivedData->radarState.CtrlRelayCfg),
                             &(receivedData->radarState.OutputTypeCfg),
                             &(receivedData->radarState.SendQualityCfg),
                             &(receivedData->radarState.SendExtInfoCfg),
                             &(receivedData->radarState.MotionRxState),
                             &(receivedData->radarState.RCS_Threshold));
        //        printf("(0x201) RCS_Threshold: %d \n",receivedData->radarState.RCS_Threshold);
        //        printf("(0x201) RadarPowerCfg: %d \n",receivedData->radarState.RadarPowerCfg);
        //        printf("(0x201) MaxDistanceCfg: %d \n",receivedData->radarState.MaxDistanceCfg);
        //        printf("(0x201) SortIndex: %d \n",receivedData->radarState.SortIndex);
        //        printf("(0x201) MotionRxState: %d \n",receivedData->radarState.MotionRxState);
        //        printf("(0x201) CtrlRelayCfg: %d \n",receivedData->radarState.CtrlRelayCfg);


        break;

    case 1792:  // (0x700) CAN1_VersionID
        break;
        
    case 1536:  // (0x600) Cluster_0_Status cluster list header information
        break;
    case 1546: // (0x60A) Object list status (ID 0)
        //printf("(0x60A) Ha recibido TOTAL %d 0x60B=%d 0x60C=%d 0x60D=%d objetos \n", receivedData->status.NumOfObjects, receivedData->status.receivedNumOfObjects1, receivedData->status.receivedNumOfObjects2, receivedData->status.receivedNumOfObjects3 );

        borrarEstructura(receivedData);
        receivedData->status.RadarID = 0;
        read_60A_Object_0_Status_Message(aux, &(receivedData->status.NumOfObjects), &(receivedData->status.MeasCounter), &(receivedData->status.InterfaceVersion));
        //printf("(0x60A) Espera %d objetos seq %d \n", receivedData->status.NumOfObjects, receivedData->status.MeasCounter);
        break;

    case 1547: // (0x60B) //Object general information (ID 0)
        receivedData->status.RadarID = 0;
        if (receivedData->status.receivedNumOfObjects1 > receivedData->status.NumOfObjects)
        {
            printf("ARS408 B: 1 ERROR: received data: %d/%d  \n", receivedData->status.receivedNumOfObjects1, receivedData->status.NumOfObjects);
        }
        else
        {
            read_60B_Object_1_General_Message(aux, &Obj_ID, &Obj_LongDispl, &Obj_LatDispl, &Obj_VrelLong, &Object_DynProp, &Obj_LatSpeed, &Obj_RCSValue);
            
            //printf("\n Obj_ID:%d\n", Obj_ID);
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_ID = Obj_ID;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_LongDispl = Obj_LongDispl;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_LatDispl = Obj_LatDispl;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_VrelLong = Obj_VrelLong;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Object_DynProp = Object_DynProp;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_LatSpeed = Obj_LatSpeed;
            receivedData->objects[receivedData->status.receivedNumOfObjects1].Obj_RCSValue = Obj_RCSValue;
            (receivedData->status.receivedNumOfObjects1)++;

            if(m_radarConfig == ENABLE_60B)
            {
                //printf("(0x60B) Objeto %d de %d, ID: %d\n", receivedData->status.receivedNumOfObjects1, receivedData->status.NumOfObjects, Obj_ID);
                if (receivedData->status.receivedNumOfObjects1 == receivedData->status.NumOfObjects)
                    // Ya se han recibido todos los datos de este radar.
                {
                    receivedData->status.completed = true;
                    //   printf("1 BIEN: received objects: %d  \n", receivedData->status.NumOfObjects);
                }
            }


        }
        break;

    case 1548: // (0x60C) Object quality information (ID 0)
        receivedData->status.RadarID = 0;
        if (receivedData->status.receivedNumOfObjects2 >receivedData->status.NumOfObjects)
        {
            printf("ARS408 C: 2 ERROR: received data: %d/%d \n", receivedData->status.receivedNumOfObjects2, receivedData->status.NumOfObjects);
        }
        else
        {
            read_60C_Object_2_Quality_Information(aux, &Obj_ID, &Obj_DistLong_rms, &Obj_VrelLong_rms, &Obj_DistLat_rms, &Obj_VrelLat_rms, &Obj_ArelLat_rms, &Obj_ArelLong_rms,
                                                  &Obj_Orientation_rms, &Obj_MeasState, &Obj_ProbOfExist);

            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_DistLong_rms = Obj_DistLong_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_VrelLong_rms = Obj_VrelLong_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_DistLat_rms = Obj_DistLat_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_VrelLat_rms = Obj_VrelLat_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_ArelLat_rms = Obj_ArelLat_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_ArelLong_rms = Obj_ArelLong_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_Orientation_rms = Obj_Orientation_rms;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_MeasState = Obj_MeasState;
            receivedData->objects[receivedData->status.receivedNumOfObjects2].Obj_ProbOfExist = Obj_ProbOfExist;
            (receivedData->status.receivedNumOfObjects2)++;
            //printf("(0x60C) Objeto %d de %d, ID: %d\n", receivedData->status.receivedNumOfObjects2, receivedData->status.NumOfObjects, Obj_ID);
            if(m_radarConfig ==ENABLE_60C)
            {

                if ((receivedData->status.receivedNumOfObjects1 == receivedData->status.receivedNumOfObjects2)
                        && (receivedData->status.receivedNumOfObjects2 == receivedData->status.NumOfObjects) ) // Ya se han recibido todos los datos de este radar.
                {
                    receivedData->status.completed = true;
                    //   printf("1 BIEN: received objects: %d  \n", receivedData->status.NumOfObjects);
                }
            }

        }
        break;

    case 1549: // (0x60D) Object extended information (ID 0)
        receivedData->status.RadarID = 0;
        if (receivedData->status.receivedNumOfObjects3 > receivedData->status.NumOfObjects)
        {
            printf("ARS408 D: 3 ERROR: received data: %d/%d  \n", receivedData->status.receivedNumOfObjects3, receivedData->status.NumOfObjects);
        }
        else
        {
            read_60D_Object_3_Extended_Message(aux, &Obj_ID, &Obj_ArelLong, &Obj_Class, &Obj_ArelLat, &Obj_OrientationAngle,  &Obj_Length,  &Obj_Width);

            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_ArelLong = Obj_ArelLong;
            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_Class = Obj_Class;
            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_ArelLat = Obj_ArelLat;
            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_OrientationAngle = Obj_OrientationAngle;
            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_Length = Obj_Length;
            receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_Width = Obj_Width;
            (receivedData->status.receivedNumOfObjects3)++;
            //printf("(0x60D) Objeto %d de %d, ID: %d, Width %lf \n", receivedData->status.receivedNumOfObjects3, receivedData->status.NumOfObjects, receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_ID, receivedData->objects[receivedData->status.receivedNumOfObjects3].Obj_Width);
        }
        if(m_radarConfig ==ENABLE_60D)
        {
            if (   (receivedData->status.receivedNumOfObjects1 == receivedData->status.receivedNumOfObjects2)
                   && (receivedData->status.receivedNumOfObjects2 == receivedData->status.receivedNumOfObjects3)
                   && (receivedData->status.receivedNumOfObjects3 == receivedData->status.NumOfObjects) ) // Ya se han recibido todos los datos de este radar.
            {
                receivedData->status.completed = true;
                //  printf("BIEN: received objects: %d  \n", receivedData->status.NumOfObjects);
            }
        }else if(m_radarConfig ==ENABLE_60BD)
        {
            if (   (receivedData->status.receivedNumOfObjects1 == receivedData->status.receivedNumOfObjects3)
                   && (receivedData->status.receivedNumOfObjects3 == receivedData->status.NumOfObjects) ) // Ya se han recibido todos los datos de este radar.
            {
                receivedData->status.completed = true;
                //  printf("BIEN: received objects: %d  \n", receivedData->status.NumOfObjects);
            }
        }

        break;

    case 1550: // (0x60E) Object collision detection warning (ID 0)
        read_60E_Object_4_Warning_Message(aux, &basura);
        break;

    default:
        retValue = -1;
        break;
    }


    return retValue;
}

void Radar_ARS408::read_Message_ID(/*in:*/ char* aux,
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

//Radar_State (0x201)
void Radar_ARS408::read_201_Radar_State( /*in:*/ char* aux,
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
                                   bool*    RCS_Threshold)
{
    char oneByte[2];
    oneByte[1]='\0';

    char twoByte[3];
    twoByte[2]='\0';

    char fourByte[5];
    fourByte[4]='\0';

    int auxInt = 0;

    oneByte[0] = aux[6];
    auxInt = strtoul(oneByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x1;
    *NVMReadStatus = (bool)auxInt;
    auxInt = strtoul(oneByte,NULL,16);
    auxInt = auxInt >> 3;
    auxInt &= 0x1;
    *NVMwriteStatus = (bool)auxInt;

    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 6;
    auxInt &= 0x03FF;
    *MaxDistanceCfg = ((int)auxInt)*2;

    twoByte[0] = aux[9];
    twoByte[1] = aux[10];
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 1;
    auxInt &= 0x01;
    *Voltage_Error = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x01;
    *Temporary_Error = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 3;
    auxInt &= 0x01;
    *Temperature_Error = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x01;
    *Interference = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x01;
    *Persistent_Error = (bool)auxInt;

    fourByte[0] = aux[11];
    fourByte[1] = aux[12];
    fourByte[2] = aux[13];
    fourByte[3] = aux[14];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 7;
    auxInt &= 0x0007;
    *RadarPowerCfg = (int)auxInt;
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x0007;
    *SortIndex = (int)auxInt;
    auxInt = strtoul(fourByte,NULL,16);
    auxInt &= 0x0007;
    *SensorID = (int)auxInt;

    twoByte[0] = aux[15];
    twoByte[1] = aux[16];
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 1;
    auxInt &= 0x01;
    *CtrlRelayCfg = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x03;
    *OutputTypeCfg = (int)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x01;
    *SendQualityCfg = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 5;
    auxInt &= 0x01;
    *SendExtInfoCfg = (bool)auxInt;
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 6;
    auxInt &= 0x03;
    *MotionRxState = (int)auxInt;

    twoByte[0] = aux[19];
    twoByte[1] = aux[20];
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x07;
    *RCS_Threshold = (int)auxInt;
}

//Object list status (0x60A)
void Radar_ARS408::read_60A_Object_0_Status_Message(/*in:*/ char* aux,
                                                   /*out:*/ int* NumOfObjects,
                                                            int* MeasCounter,
                                                            int* InterfaceVersion)
{
    char oneByte[2];
    oneByte[1]='\0';

    char twoByte[3];
    twoByte[2]='\0';

    char fourByte[5];
    fourByte[4]='\0';

    twoByte[0] = aux[5];
    twoByte[1] = aux[6];
    *NumOfObjects = strtoul(twoByte,NULL,16);

    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    *MeasCounter = strtoul(fourByte,NULL,16);

    oneByte[0] = aux[12];
    *InterfaceVersion = strtoul(oneByte,NULL,16);
}

//Object general information (0x60B)
void Radar_ARS408::read_60B_Object_1_General_Message(/*in:*/  char* aux,
                                                     /*out:*/ int* Object_ID,
                                                              double* Object_DistLong,
                                                              double* Object_DistLat,
                                                              double* Object_VrelLong,
                                                              t_DynProp* Object_DynProp,
                                                              double* Object_VrelLat,
                                                              double* Object_RCS)
{
//     printf("\n\nReceived %s\n", aux);
    char oneByte[2];
//     printf("\n\nReceived %s\n", aux);
    oneByte[1]='\0';
//     printf("\n\nReceived %s\n", aux);    
    char twoByte[3];
//     printf("\n\nReceived %s\n", aux);    
    twoByte[2]='\0';
//     printf("\n\nReceived %s\n", aux);    
    char fourByte[5];
//     printf("\n\nReceived %s\n", aux);    
    fourByte[4]='\0';
//     printf("\n\nReceived %s\n", aux);
    twoByte[0] = aux[5];
//     printf("\n\nReceived %s\n", aux); 
    twoByte[1] = aux[6];
//     printf("\n\nReceived %s\n", aux); 
    *Object_ID = strtoul(twoByte,NULL,16);
//     printf("\n\nReceived %s with ObjectID:%d\n", aux, *Object_ID); 
 

    int int_Object_DistLong;
    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    int_Object_DistLong = strtoul(fourByte,NULL,16);
    int_Object_DistLong = int_Object_DistLong >> 3;
    int_Object_DistLong &= 0x1FFF;
    *Object_DistLong = (((double)int_Object_DistLong)*0.2)-500.0;

    int int_Object_DistLat;
    fourByte[0] = aux[9];
    fourByte[1] = aux[10];
    fourByte[2] = aux[11];
    fourByte[3] = aux[12];
    int_Object_DistLat = strtoul(fourByte,NULL,16);
    int_Object_DistLat &= 0x07FF;
    *Object_DistLat = (((double)int_Object_DistLat)*0.2)-204.6;


// //     printf("\n\nReceived %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c with ObjectID:%d Object_DistLong %lf Object_DistLat %lf\n", aux[0], aux[1],aux[2],aux[3],aux[4],aux[5],aux[6],aux[7],aux[8],aux[9],aux[10],aux[11],aux[12],aux[13],aux[14],aux[15],aux[16],aux[17],aux[18],aux[19],aux[20],*Object_ID, 
// //     *Object_DistLong, *Object_DistLat);
    
    int int_Object_VrelLong;
    fourByte[0] = aux[13];
    fourByte[1] = aux[14];
    fourByte[2] = aux[15];
    fourByte[3] = aux[16];
    int_Object_VrelLong = strtoul(fourByte,NULL,16);
    int_Object_VrelLong = int_Object_VrelLong >> 6;
    int_Object_VrelLong &= 0x03FF;
    *Object_VrelLong = (((double)int_Object_VrelLong)*0.25)-128.0;

    int int_Object_VrelLat;
    fourByte[0] = aux[15];
    fourByte[1] = aux[16];
    fourByte[2] = aux[17];
    fourByte[3] = aux[18];
    int_Object_VrelLat = strtoul(fourByte,NULL,16);
    int_Object_VrelLat = int_Object_VrelLat >> 5;
    int_Object_VrelLat &= 0x01FF;
    *Object_VrelLat = (((double)int_Object_VrelLat)*0.25)-64.0;

    int int_Object_DynProp;
    oneByte[0] = aux[18];
    int_Object_DynProp = strtoul(oneByte,NULL,16);
    int_Object_DynProp &= 0x7;
    //{moving, stationary, oncoming, stationary_candidate, unknown, crossing_stationary, crossing_moving, stopped};
    switch (int_Object_DynProp)
    {
        case 0: *Object_DynProp = moving; break;
        case 1: *Object_DynProp = stationary; break;
        case 2: *Object_DynProp = oncoming; break;
        case 3: *Object_DynProp = stationary_candidate; break;
        case 4: *Object_DynProp = DynProp_unknown; break;
        case 5: *Object_DynProp = crossing_stationary; break;
        case 6: *Object_DynProp = crossing_moving; break;
        case 7: *Object_DynProp = stopped; break;
    }


    int int_Object_RCS;
    twoByte[0] = aux[19];
    twoByte[1] = aux[20];
    int_Object_RCS = strtoul(twoByte,NULL,16);
    *Object_RCS = (((double)int_Object_RCS)*0.5)-64.0;
}

//Object quality information (0x60C)
void Radar_ARS408::read_60C_Object_2_Quality_Information(  /*in:*/ char* aux,
                                                           /*out:*/ int*     Obj_ID, // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
                                                                    double*  Obj_DistLong_rms, //Standard deviation of longitudinal distance
                                                                    double*  Obj_VrelLong_rms, //Standard deviation of longitudinal relative velocity
                                                                    double*  Obj_DistLat_rms, //Standard deviation of lateral distance
                                                                    double*  Obj_VrelLat_rms, //Standard deviation of lateral relative velocity
                                                                    double*  Obj_ArelLat_rms, //Standard deviation of lateral relative acceleration
                                                                    double*  Obj_ArelLong_rms, //Standard deviation of longitudinal relative acceleration
                                                                    double*  Obj_Orientation_rms, //Standard deviation of orientation angle
                                                                    t_MeasState*  Obj_MeasState, //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
                                                                    int*     Obj_ProbOfExist) //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%)
{
    char oneByte[2];
    oneByte[1]='\0';
    char twoByte[3];
    twoByte[2]='\0';
    char fourByte[5];
    fourByte[4]='\0';

    int auxInt = 0;

    twoByte[0] = aux[5];
    twoByte[1] = aux[6];
    *Obj_ID = strtoul(twoByte,NULL,16);

    twoByte[0] = aux[7];
    twoByte[1] = aux[8];
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 3;
    auxInt &= 0x1F;
    *Obj_DistLong_rms = auxInt2StdDeviation(auxInt);

    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 6;
    auxInt &= 0x001F;
    *Obj_DistLat_rms = auxInt2StdDeviation(auxInt);
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 1;
    auxInt &= 0x001F;
    *Obj_VrelLong_rms = auxInt2StdDeviation(auxInt);

    fourByte[0] = aux[9];
    fourByte[1] = aux[10];
    fourByte[2] = aux[11];
    fourByte[3] = aux[12];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x001F;
    *Obj_VrelLat_rms = auxInt2StdDeviation(auxInt);

    fourByte[0] = aux[11];
    fourByte[1] = aux[12];
    fourByte[2] = aux[13];
    fourByte[3] = aux[14];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 7;
    auxInt &= 0x001F;
    *Obj_ArelLong_rms = auxInt2StdDeviation(auxInt);
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x001F;
    *Obj_ArelLat_rms = auxInt2StdDeviation(auxInt);

    fourByte[0] = aux[13];
    fourByte[1] = aux[14];
    fourByte[2] = aux[15];
    fourByte[3] = aux[16];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 5;
    auxInt &= 0x001F;
    *Obj_Orientation_rms = auxInt2StdDeviationDegrees(auxInt);

    twoByte[0] = aux[17];
    twoByte[1] = aux[18];
    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 5;
    auxInt &= 0x07;
    //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%
    *Obj_ProbOfExist = auxInt;

    auxInt = strtoul(twoByte,NULL,16);
    auxInt = auxInt >> 2;
    auxInt &= 0x07;
    //enum t_MeasState  {Deleted = 0, New = 1, Measured = 2, Predicted = 3, Deleted_for_merge = 4, New_from_merge = 5};
    switch (auxInt)
    {
        case 0: *Obj_MeasState = Deleted; break;
        case 1: *Obj_MeasState = New; break;
        case 2: *Obj_MeasState = Measured; break;
        case 3: *Obj_MeasState = Predicted; break;
        case 4: *Obj_MeasState = Deleted_for_merge; break;
        case 5: *Obj_MeasState = New_from_merge; break;
    }
}

double Radar_ARS408::auxInt2StdDeviation(int auxInt)
{
    double auxDouble = -1.0;
    switch (auxInt)
    {
        case 0: auxDouble = 0.005; break;
        case 1: auxDouble = 0.006; break;
        case 2: auxDouble = 0.008; break;
        case 3: auxDouble = 0.011; break;
        case 4: auxDouble = 0.014; break;
        case 5: auxDouble = 0.018; break;
        case 6: auxDouble = 0.023; break;
        case 7: auxDouble = 0.029; break;
        case 8: auxDouble = 0.038; break;
        case 9: auxDouble = 0.049; break;
        case 10: auxDouble = 0.063; break;
        case 11: auxDouble = 0.081; break;
        case 12: auxDouble = 0.105; break;
        case 13: auxDouble = 0.135; break;
        case 14: auxDouble = 0.174; break;
        case 15: auxDouble = 0.224; break;
        case 16: auxDouble = 0.288; break;
        case 17: auxDouble = 0.371; break;
        case 18: auxDouble = 0.478; break;
        case 19: auxDouble = 0.616; break;
        case 20: auxDouble = 0.794; break;
        case 21: auxDouble = 1.023; break;
        case 22: auxDouble = 1.317; break;
        case 23: auxDouble = 1.697; break;
        case 24: auxDouble = 2.187; break;
        case 25: auxDouble = 2.817; break;
        case 26: auxDouble = 3.630; break;
        case 27: auxDouble = 4.676; break;
        case 28: auxDouble = 6.025; break;
        case 29: auxDouble = 7.762; break;
        case 30: auxDouble = 10.000; break;
        case 31: auxDouble = -1.0; break;//invalid
    }
    return auxDouble;
}

double Radar_ARS408::auxInt2StdDeviationDegrees(int auxInt)
{
    double auxDouble = -1.0;
    switch (auxInt)
    {
        case 0: auxDouble = 0.005; break;
        case 1: auxDouble = 0.007; break;
        case 2: auxDouble = 0.010; break;
        case 3: auxDouble = 0.014; break;
        case 4: auxDouble = 0.020; break;
        case 5: auxDouble = 0.029; break;
        case 6: auxDouble = 0.041; break;
        case 7: auxDouble = 0.058; break;
        case 8: auxDouble = 0.082; break;
        case 9: auxDouble = 0.116; break;
        case 10: auxDouble = 0.165; break;
        case 11: auxDouble = 0.234; break;
        case 12: auxDouble = 0.332; break;
        case 13: auxDouble = 0.471; break;
        case 14: auxDouble = 0.669; break;
        case 15: auxDouble = 0.949; break;
        case 16: auxDouble = 1.346; break;
        case 17: auxDouble = 1.909; break;
        case 18: auxDouble = 2.709; break;
        case 19: auxDouble = 3.843; break;
        case 20: auxDouble = 5.451; break;
        case 21: auxDouble = 7.734; break;
        case 22: auxDouble = 10.971; break;
        case 23: auxDouble = 15.565; break;
        case 24: auxDouble = 22.081; break;
        case 25: auxDouble = 31.325; break;
        case 26: auxDouble = 44.439; break;
        case 27: auxDouble = 63.044; break;
        case 28: auxDouble = 89.437; break;
        case 29: auxDouble = 126.881; break;
        case 30: auxDouble = 180.000; break;
        case 31: auxDouble = -1.0; break;//invalid
    }
    return auxDouble;
}

//Object extended information (0x60D)
void Radar_ARS408::read_60D_Object_3_Extended_Message(  /*in:*/ char*    aux,
                                                       /*out:*/ int*     Obj_ID, // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
                                                                double*  Obj_ArelLong, //Relative acceleration in longitudinal direction
                                                                t_Class* Obj_Class, //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
                                                                double*  Obj_ArelLat, //Relative acceleration in lateral direction
                                                                double*  Obj_OrientationAngle, //Orientation angle of the object
                                                                double*  Obj_Length, //Length of the tracked object
                                                                double*  Obj_Width) //Width of the tracked object)
{
    char oneByte[2];
    oneByte[1]='\0';
    char twoByte[3];
    twoByte[2]='\0';
    char fourByte[5];
    fourByte[4]='\0';

    int auxInt = 0;

    twoByte[0] = aux[5];
    twoByte[1] = aux[6];
    *Obj_ID = strtoul(twoByte,NULL,16);

    fourByte[0] = aux[7];
    fourByte[1] = aux[8];
    fourByte[2] = aux[9];
    fourByte[3] = aux[10];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 5;
    auxInt &= 0x07FF;
    *Obj_ArelLong = (((double)auxInt)*0.01)-10.0;

    fourByte[0] = aux[9];
    fourByte[1] = aux[10];
    fourByte[2] = aux[11];
    fourByte[3] = aux[12];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 4;
    auxInt &= 0x01FF;
    *Obj_ArelLat = (((double)auxInt)*0.01)-2.5;

    auxInt = strtoul(fourByte,NULL,16);
    auxInt &= 0x0007;
    //enum t_Class {point = 0, car = 1, truck = 2, pedestrian = 3, motorcycle = 4, bicycle = 5, wide = 6, reserved = 7};
    switch (auxInt)
    {
        case 0: *Obj_Class = point; break;
        case 1: *Obj_Class = car; break;
        case 2: *Obj_Class = truck; break;
        case 3: *Obj_Class = pedestrian; break;
        case 4: *Obj_Class = motorcycle; break;
        case 5: *Obj_Class = bicycle; break;
        case 6: *Obj_Class = wide; break;
        case 7: *Obj_Class = reserved; break;
    }

    fourByte[0] = aux[13];
    fourByte[1] = aux[14];
    fourByte[2] = aux[15];
    fourByte[3] = aux[16];
    auxInt = strtoul(fourByte,NULL,16);
    auxInt = auxInt >> 6;
    auxInt &= 0x03FF;
    *Obj_OrientationAngle = (((double)auxInt)*0.4)-180.0;

    twoByte[0] = aux[17];
    twoByte[1] = aux[18];
    auxInt = strtoul(twoByte,NULL,16);
    *Obj_Length = ((double)auxInt)*0.2;

    twoByte[0] = aux[19];
    twoByte[1] = aux[20];
    auxInt = strtoul(twoByte,NULL,16);
    *Obj_Width = ((double)auxInt)*0.2;
}

//Object collision detection warning (0x60E)
//Object extended information (0x60D)
void Radar_ARS408::read_60E_Object_4_Warning_Message(  /*in:*/ char* aux,
                                                       /*out:*/ int* nada)
{
    *nada = 0;
}

void Radar_ARS408::borrarEstructura(Radar_ARS408_data_t* data)
{
    data->status.NumOfObjects          = 0;
    data->status.MeasCounter           = 0;
    data->status.InterfaceVersion      = 0;
    data->status.RadarID               = 0;
    data->status.receivedNumOfObjects1 = 0;
    data->status.receivedNumOfObjects2 = 0;
    data->status.receivedNumOfObjects3 = 0;
    data->status.completed             = false;

    //enum t_DynProp {moving = 0, stationary = 1, oncoming = 2, stationary_candidate = 3, DynProp_unknown = 4, crossing_stationary = 5, crossing_moving = 6, stopped = 7};
    //enum t_MeasState  {Deleted = 0, New = 1, Measured = 2, Predicted = 3, Deleted_for_merge = 4, New_from_merge = 5, MeasState_unknown = 6};
    //enum t_Class {point = 0, car = 1, truck = 2, pedestrian = 3, motorcycle = 4, bicycle = 5, wide = 6, reserved = 7, Class_unknown = 8};

    for (int i = 0; i++; i<MAX_OBJECTS_408)
    {
        data->objects[i].Obj_ID             = 0; // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)

        //0x60B
        data->objects[i].Obj_LongDispl      = 0; //Longitudinal displacement
        data->objects[i].Obj_LatDispl       = 0;    //Lateral displacement
        data->objects[i].Obj_VrelLong       = 0;    //Relative longitudinal speed
        data->objects[i].Object_DynProp     = DynProp_unknown; //Dynamic property of the object indicating if the object is moving or stationary (this value can only be determined correctly if the speed and yaw rate is given correctly)
        data->objects[i].Obj_LatSpeed       = 0;    //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)
        data->objects[i].Obj_RCSValue       = 0;    //Radar cross section (RCS) is the measure of the reflective strength of a target or in other words it is the measure of power scattered in a given spatial direction when a target is illuminated by an incident wave.

        //0x60C
        data->objects[i].Obj_DistLong_rms   = 0.0; //Standard deviation of longitudinal distance
        data->objects[i].Obj_VrelLong_rms   = 0.0; //Standard deviation of longitudinal relative velocity
        data->objects[i].Obj_DistLat_rms    = 0.0; //Standard deviation of lateral distance
        data->objects[i].Obj_VrelLat_rms    = 0.0; //Standard deviation of lateral relative velocity
        data->objects[i].Obj_ArelLat_rms    = 0.0; //Standard deviation of lateral relative acceleration
        data->objects[i].Obj_ArelLong_rms   = 0.0; //Standard deviation of longitudinal relative acceleration
        data->objects[i].Obj_Orientation_rms = 0.0; //Standard deviation of orientation angle
        data->objects[i].Obj_MeasState      = MeasState_unknown; //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
        data->objects[i].Obj_ProbOfExist    = 0.0; //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%

        //0x60D
        data->objects[i].Obj_ArelLong       = 0; //Relative acceleration in longitudinal direction
        data->objects[i].Obj_Class          = Class_unknown; //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
        data->objects[i].Obj_ArelLat        = 0; //Relative acceleration in lateral direction
        data->objects[i].Obj_OrientationAngle = 0; //Orientation angle of the object
        data->objects[i].Obj_Length         = 0; //Length of the tracked object
        data->objects[i].Obj_Width          = 0; //Width of the tracked object
    }
}

void Radar_ARS408::borrarEstructura()
{
    m_dataRadar_ARS.status.NumOfObjects          = 0;
    m_dataRadar_ARS.status.MeasCounter           = 0;
    m_dataRadar_ARS.status.InterfaceVersion      = 0;
    m_dataRadar_ARS.status.RadarID               = 0;
    m_dataRadar_ARS.status.receivedNumOfObjects1 = 0;
    m_dataRadar_ARS.status.receivedNumOfObjects2 = 0;
    m_dataRadar_ARS.status.receivedNumOfObjects3 = 0;
    m_dataRadar_ARS.status.completed             = false;

    //enum t_DynProp {moving = 0, stationary = 1, oncoming = 2, stationary_candidate = 3, DynProp_unknown = 4, crossing_stationary = 5, crossing_moving = 6, stopped = 7};
    //enum t_MeasState  {Deleted = 0, New = 1, Measured = 2, Predicted = 3, Deleted_for_merge = 4, New_from_merge = 5, MeasState_unknown = 6};
    //enum t_Class {point = 0, car = 1, truck = 2, pedestrian = 3, motorcycle = 4, bicycle = 5, wide = 6, reserved = 7, Class_unknown = 8};

    for (int i = 0; i++; i<MAX_OBJECTS_408)
    {
        m_dataRadar_ARS.objects[i].Obj_ID             = 0; // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)

        //0x60B
        m_dataRadar_ARS.objects[i].Obj_LongDispl      = 0; //Longitudinal displacement
        m_dataRadar_ARS.objects[i].Obj_LatDispl       = 0;    //Lateral displacement
        m_dataRadar_ARS.objects[i].Obj_VrelLong       = 0;    //Relative longitudinal speed
        m_dataRadar_ARS.objects[i].Object_DynProp     = DynProp_unknown; //Dynamic property of the object indicating if the object is moving or stationary (this value can only be determined correctly if the speed and yaw rate is given correctly)
        m_dataRadar_ARS.objects[i].Obj_LatSpeed       = 0;    //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)
        m_dataRadar_ARS.objects[i].Obj_RCSValue       = 0;    //Radar cross section (RCS) is the measure of the reflective strength of a target or in other words it is the measure of power scattered in a given spatial direction when a target is illuminated by an incident wave.

        //0x60C
        m_dataRadar_ARS.objects[i].Obj_DistLong_rms   = 0.0; //Standard deviation of longitudinal distance
        m_dataRadar_ARS.objects[i].Obj_VrelLong_rms   = 0.0; //Standard deviation of longitudinal relative velocity
        m_dataRadar_ARS.objects[i].Obj_DistLat_rms    = 0.0; //Standard deviation of lateral distance
        m_dataRadar_ARS.objects[i].Obj_VrelLat_rms    = 0.0; //Standard deviation of lateral relative velocity
        m_dataRadar_ARS.objects[i].Obj_ArelLat_rms    = 0.0; //Standard deviation of lateral relative acceleration
        m_dataRadar_ARS.objects[i].Obj_ArelLong_rms   = 0.0; //Standard deviation of longitudinal relative acceleration
        m_dataRadar_ARS.objects[i].Obj_Orientation_rms = 0.0; //Standard deviation of orientation angle
        m_dataRadar_ARS.objects[i].Obj_MeasState      = MeasState_unknown; //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
        m_dataRadar_ARS.objects[i].Obj_ProbOfExist    = 0.0; //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%

        //0x60D
        m_dataRadar_ARS.objects[i].Obj_ArelLong       = 0; //Relative acceleration in longitudinal direction
        m_dataRadar_ARS.objects[i].Obj_Class          = Class_unknown; //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
        m_dataRadar_ARS.objects[i].Obj_ArelLat        = 0; //Relative acceleration in lateral direction
        m_dataRadar_ARS.objects[i].Obj_OrientationAngle = 0; //Orientation angle of the object
        m_dataRadar_ARS.objects[i].Obj_Length         = 0; //Length of the tracked object
        m_dataRadar_ARS.objects[i].Obj_Width          = 0; //Width of the tracked object
    }
}

void Radar_ARS408::parse_radar_msg_408(const radar_msgs::msg::RadarRaw msgIn)
{
    auto msg = radar_msgs::msg::RadarMsg408(); 
    char aux[23];

    memcpy(aux, (const char*) (&(msgIn.raw[0])), 22);
    aux[23]='\0';
    
    RCLCPP_DEBUG(rclcpp::get_logger("radar"), "I heard: [%d:%d  %s]", msgIn.header.stamp.sec,msgIn.header.stamp.nanosec,aux);

    read_Radar(aux, &m_dataRadar_ARS);
    
    if(m_dataRadar_ARS.status.completed)
    {
        m_dataRadar_ARS.status.completed = false;
        //ROS_INFO(" COMPLETADO ");
        msg.header.stamp = msgIn.header.stamp;
        msg.num_of_objects = m_dataRadar_ARS.status.NumOfObjects;
        msg.meas_counter = m_dataRadar_ARS.status.MeasCounter;
        msg.interface_version      = m_dataRadar_ARS.status.InterfaceVersion;
        msg.radar_id               = m_dataRadar_ARS.status.RadarID;
        msg.received_num_of_objects1 = m_dataRadar_ARS.status.receivedNumOfObjects1;
        msg.received_num_of_objects2 = m_dataRadar_ARS.status.receivedNumOfObjects2;
        msg.received_num_of_objects3 = m_dataRadar_ARS.status.receivedNumOfObjects3;
        msg.completed             = m_dataRadar_ARS.status.completed;
        
        for (int i=0;i<m_dataRadar_ARS.status.NumOfObjects;i++)
        {
            msg.objects[i].obj_id= m_dataRadar_ARS.objects[i].Obj_ID;

            msg.objects[i].obj_u = m_dataRadar_ARS.objects[i].Obj_u;
            msg.objects[i].obj_v = m_dataRadar_ARS.objects[i].Obj_v;
            
            msg.objects[i].obj_long_displ= m_dataRadar_ARS.objects[i].Obj_LongDispl;
            msg.objects[i].obj_lat_displ= m_dataRadar_ARS.objects[i].Obj_LatDispl;            
            msg.objects[i].obj_vrel_long= m_dataRadar_ARS.objects[i].Obj_VrelLong;
            msg.objects[i].obj_long_displ_cam= m_dataRadar_ARS.objects[i].Obj_LongDispl_cam;            
            msg.objects[i].obj_lat_displ_cam= m_dataRadar_ARS.objects[i].Obj_LatDispl_cam;
            msg.objects[i].z_cam= m_dataRadar_ARS.objects[i].z_cam;            
            msg.objects[i].obj_vrel_long_cam= m_dataRadar_ARS.objects[i].Obj_VrelLong_cam;
            msg.objects[i].object_dyn_prop= m_dataRadar_ARS.objects[i].Object_DynProp;
            
            msg.objects[i].obj_lat_speed= m_dataRadar_ARS.objects[i].Obj_LatSpeed;
            msg.objects[i].obj_lat_speed_cam= m_dataRadar_ARS.objects[i].Obj_LatSpeed_cam;
            msg.objects[i].obj_rcs_value= m_dataRadar_ARS.objects[i].Obj_RCSValue;

            msg.objects[i].obj_dist_long_rms= m_dataRadar_ARS.objects[i].Obj_DistLong_rms;
            msg.objects[i].obj_vrel_long_rms= m_dataRadar_ARS.objects[i].Obj_VrelLong_rms;
            msg.objects[i].obj_dist_lat_rms= m_dataRadar_ARS.objects[i].Obj_DistLat_rms;
            msg.objects[i].obj_vrel_lat_rms= m_dataRadar_ARS.objects[i].Obj_VrelLat_rms;
            msg.objects[i].obj_arel_lat_rms= m_dataRadar_ARS.objects[i].Obj_ArelLat_rms;
            msg.objects[i].obj_arel_long_rms= m_dataRadar_ARS.objects[i].Obj_ArelLong_rms;
            msg.objects[i].obj_orientation_rms= m_dataRadar_ARS.objects[i].Obj_Orientation_rms;
            msg.objects[i].obj_meas_state= m_dataRadar_ARS.objects[i].Obj_MeasState;
            msg.objects[i].obj_prob_of_exist= m_dataRadar_ARS.objects[i].Obj_ProbOfExist;            
            
            msg.objects[i].obj_arel_long= m_dataRadar_ARS.objects[i].Obj_ArelLong;
            msg.objects[i].obj_class= m_dataRadar_ARS.objects[i].Obj_Class;
            msg.objects[i].obj_arel_lat= m_dataRadar_ARS.objects[i].Obj_ArelLat;
            msg.objects[i].obj_orientation_angle= m_dataRadar_ARS.objects[i].Obj_OrientationAngle;
            msg.objects[i].obj_dist_long_rms= m_dataRadar_ARS.objects[i].Obj_DistLong_rms;
            msg.objects[i].obj_length= m_dataRadar_ARS.objects[i].Obj_Length;
        }

        msg.nvm_read_status     = m_dataRadar_ARS.radarState.NVMReadStatus;
        msg.nvm_write_status    = m_dataRadar_ARS.radarState.NVMwriteStatus;
        msg.voltage_error       = m_dataRadar_ARS.radarState.Voltage_Error;
        msg.temporary_error     = m_dataRadar_ARS.radarState.Temporary_Error;
        msg.temperature_error   = m_dataRadar_ARS.radarState.Temperature_Error;
        msg.interference        = m_dataRadar_ARS.radarState.Interference;
        msg.persistent_error    = m_dataRadar_ARS.radarState.Persistent_Error;
        msg.max_distance_cfg    = m_dataRadar_ARS.radarState.MaxDistanceCfg;
        msg.sensor_id           = m_dataRadar_ARS.radarState.SensorID;
        msg.sort_index          = m_dataRadar_ARS.radarState.SortIndex;
        msg.radar_power_cfg     = m_dataRadar_ARS.radarState.RadarPowerCfg;
        msg.ctrl_relay_cfg      = m_dataRadar_ARS.radarState.CtrlRelayCfg;
        msg.output_type_cfg     = m_dataRadar_ARS.radarState.OutputTypeCfg;
        msg.send_quality_cfg    = m_dataRadar_ARS.radarState.SendQualityCfg;
        msg.send_ext_info_cfg   = m_dataRadar_ARS.radarState.SendExtInfoCfg;
        msg.motion_rx_state     = m_dataRadar_ARS.radarState.MotionRxState;
        msg.rcs_threshold       = m_dataRadar_ARS.radarState.RCS_Threshold;
        m_publisher->publish(msg);
    }
}
