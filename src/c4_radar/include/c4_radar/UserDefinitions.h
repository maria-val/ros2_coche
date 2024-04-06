////////////////////////////////
// SDK Programmer samples
////////////////////////////////

#ifndef _UserDefinitions_H
#define _UserDefinitions_H

//#define RTMAPS 1

#ifdef RTMAPS
	#include "maps.hpp"
#endif

#include <stdint.h>

 /*	The user-defined structure shared header.
 *	This header may be included by all of the components or rrms that exchange
 *	data of type "UAH_Obstacles_Structure". This file define both :
 *		- the C structure containing the data (no pointer must be used)
 *		- the filter to use with inputs and RRMs
 */



// Define a user-specifi structure called MyNewStructure (no pointer are allowed in user-defined structures)

// First of all, let's disable structure fields alignment (force alignment to 1 byte)
// so that the sizeof(UAH_Obstacles_Structure) will be the same whatever the platform and compiler.
// This will allow to use a generic record/replay method for all specific structures which can be used to
// record in binary format on one platform and replay on another.
#pragma pack(push,1)

#define MAX_OBJECTS_208 25
#define MAX_OBJECTS_308 40
#define MAX_OBJECTS_408 100

//#define pi 3.14159

//////////////////////////////////////////////////////////// ARS 308-2 ////////////////////////////////////////////////
struct _ObjectStatus_ARS308_data_t
{
    int NumOfNEWObjects; //Number of measured Objects (which have Obj_MeasStat new or measured. In the listing also are old objects, not be measured in this cycle, but tracked – so the quantity in listing can be higher than the NoOfObjects).
    int MeasCounter; //Counts the number of measurements made, normally increases 1 per cycle,
    int InterfaceVersionNumber; //Unique number that identifies the version of the CAN interface;
    int RadarID;
    int receivedNumOfObjects1;
    int receivedNumOfObjects2;
    bool completed;
    // Carlota
    unsigned long long writingTime;
};
typedef struct _ObjectStatus_ARS308_data_t ObjectStatus_ARS308_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterObjectStatus_ARS308_data_t = MAPS_FILTER_USER_STRUCTURE(ObjectStatus_ARS308_data_t);
#endif

struct _Objects_ARS308_data_t
{
    int         Obj_RolCount;    //Rolling counter, shows CAN activity by increasing 1 per cycle after each sent CAN cycle
    int         Obj_ID;          //Object ID
    double      Obj_LongDispl;  //Longitudinal displacement
    double      Obj_VrelLong;   //Relative longitudinal speed
    double      Obj_AccelLong;  //Relative longitudinal acceleration
    int         Obj_ProbOfExist; //Probability of existence calculated for an Object (0:invalid, 1:<25%, 2:<50%, 3:<75%, 4:<90%, 5:<99%, 6:<99.9%, 7:<99.99%)
    int         Obj_DynProp;     //Dynamic property (0:unclassified, 1:standing (has never been moving before – from static targets), 2: stopped (has been moving before), 3: moving, 4: oncoming)
    double      Obj_LatDispl;   //Lateral displacement
    double      Obj_Length;      //Object length (0:unknown, 1:<0.5m, 2:<2m, 3:<4m, 4:<6m, 5:<10m, 6:<20m, 7:exceeds
    double      Obj_Width;       //Object width (0:unknown, 1:<0.5m(pedestrian), 2:<1m(bike), 3:<2m(car), 4:<3m(truck), 5:<4m, 6:<6m, 7:exceeds)
    int         Obj_MeasStat;    //Object measurement status (0:no object, 1:new object, 2:object not measured, 3:object measured)

    double      Obj_RCSValue; //Radar cross section (RCS) is the measure of the reflective strength of a target or in other words it is the measure of power scattered in a
                    //given spatial direction when a target is illuminated by an incident wave.
    double      Obj_LatSpeed; //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)
    double      Obj_ObstacleProbability; //Probability that the Object is an Obstacle – normally for moving radar sensor depending on position, motion, size.
                    //The obstacle is not clear defined in size and characteristics, because the feasibility of a collision in algorithm is implemented for it.
                    //Here e.g. a bridge pier on road side or similar obstacles in the environment, which not will generate a collision, will be considered.

};
typedef struct _Objects_ARS308_data_t Objects_ARS308_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterObjects_ARS308_data_t = MAPS_FILTER_USER_STRUCTURE(Objects_ARS308_data_t);
#endif

struct _Radar_ARS308_data_t
{
    ObjectStatus_ARS308_data_t status;
    Objects_ARS308_data_t objects[MAX_OBJECTS_308];      // Son 40 objetos como máximo en modo Object
};
typedef struct _Radar_ARS308_data_t Radar_ARS308_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterRadar_ARS308_data_t = MAPS_FILTER_USER_STRUCTURE(Radar_ARS308_data_t);
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////// ARS 408-21 ////////////////////////////////////////////////
enum t_DynProp {moving = 0, stationary = 1, oncoming = 2, stationary_candidate = 3, DynProp_unknown = 4, crossing_stationary = 5, crossing_moving = 6, stopped = 7};

enum t_MeasState  {Deleted = 0, New = 1, Measured = 2, Predicted = 3, Deleted_for_merge = 4, New_from_merge = 5, MeasState_unknown = 6};
//0x0: Deleted Object – Object has been deleted – is displayed during the last cycles of transmission just before the object ID disappears.
//0x1: New Object is created – is displayed during the first cycles of transmission just after the object ID is created. Can also be checked by the character Object_MeasCounter.
//0x2: Measured – Object creation has been confirmed by the actual measurement. Cluster could be created.
//0x3: Predicted - Object creation could not be confirmed by the actual measurement. Cluster could not be created.
//0x4: Deleted for merge - Object became deleted in order to be merged with another Object.
//0x5: New from merge – new Object became created after a merge.

//0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
enum t_Class {point = 0, car = 1, truck = 2, pedestrian = 3, motorcycle = 4, bicycle = 5, wide = 6, reserved = 7, Class_unknown = 8};

struct _ObjectStatus_ARS408_data_t
{
    int NumOfObjects; //Number of objects (max. 100 Objects)
    int MeasCounter; //Measurement cycle counter (counting up since startup of sensor and restarting at 0 when > 65535)
    int InterfaceVersion; //Object list CAN interface version
    int RadarID;
    int receivedNumOfObjects1;
    int receivedNumOfObjects2;
    int receivedNumOfObjects3;
    bool    completed;
    // Carlota
    unsigned long long writingTime;
};
typedef struct _ObjectStatus_ARS408_data_t ObjectStatus_ARS408_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterObjectStatus_ARS408_data_t = MAPS_FILTER_USER_STRUCTURE(ObjectStatus_ARS408_data_t);
#endif

struct _Objects_ARS408_data_t
{
    int     Obj_ID; // Object ID (since objects are tracked, the ID is kept throughout measurement cycles and does not have to be consecutive)
    int     Obj_u;   //px
    int     Obj_v;   //px

    //0x60B
    double  Obj_LongDispl;   //Longitudinal displacement
    double  Obj_LatDispl;    //Lateral displacement
    double  Obj_VrelLong;    //Relative longitudinal speed
    double  Obj_LongDispl_cam;   //Longitudinal displacement
    double  Obj_LatDispl_cam;    //Lateral displacement
    double  z_cam;   
    double  Obj_VrelLong_cam;    //Relative longitudinal speed
    t_DynProp   Object_DynProp; //Dynamic property of the object indicating if the object is moving or stationary (this value can only be determined correctly if the speed and yaw rate is given correctly)
    double  Obj_LatSpeed;    //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)
    double  Obj_LatSpeed_cam;    //Object lateral velocity (negative value: object moves left to right, positive value: object moves right to left)   
    double  Obj_RCSValue;    //Radar cross section (RCS) is the measure of the reflective strength of a target or in other words it is the measure of power scattered in a
            //given spatial direction when a target is illuminated by an incident wave.

    //0x60C
    double  Obj_DistLong_rms; //Standard deviation of longitudinal distance
    double  Obj_VrelLong_rms; //Standard deviation of longitudinal relative velocity
    double  Obj_DistLat_rms; //Standard deviation of lateral distance
    double  Obj_VrelLat_rms; //Standard deviation of lateral relative velocity
    double  Obj_ArelLat_rms; //Standard deviation of lateral relative acceleration
    double  Obj_ArelLong_rms; //Standard deviation of longitudinal relative acceleration
    double  Obj_Orientation_rms; //Standard deviation of orientation angle
    t_MeasState  Obj_MeasState; //Measurement state indicating if the object is valid and has been confirmed by clusters in the new measurement cycle
    int    Obj_ProbOfExist; //Probability of existence  0x0: invalid  0x1: <25%  0x2: <50%  0x3: <75%  0x4: <90%  0x5: <99%  0x6: <99.9%  0x7: <=100%

    //0x60D
    double  Obj_ArelLong; //Relative acceleration in longitudinal direction
    t_Class Obj_Class; //0x0: point  0x1: car  0x2: truck  0x3: pedestrian  0x4: motorcycle  0x5: bicycle  0x6: wide  0x7: reserved
    double  Obj_ArelLat; //Relative acceleration in lateral direction
    double  Obj_OrientationAngle; //Orientation angle of the object
    double  Obj_Length; //Length of the tracked object
    double  Obj_Width; //Width of the tracked object
};
typedef struct _Objects_ARS408_data_t Objects_ARS408_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterObjects_ARS408_data_t = MAPS_FILTER_USER_STRUCTURE(Objects_ARS408_data_t);
#endif

struct _Radar_State_ARS408_data_t
{
    bool    NVMReadStatus;        //State of reading the configuration parameters from non-volatile memory at startup
    bool    NVMwriteStatus;       //State of storing a configuration parameter to non-volatile memory
                                  //(initially this value is set to 0x0 and set to 0x1 after a configuration has been sent and successfully stored)
    bool    Voltage_Error;        //Error will be active if the operating voltage is below or above the defined range for more than 5 seconds.
    bool    Temporary_Error;      //A temporary error which will most probably disappear after a sensor reset has been detected.
    bool    Temperature_Error;    //Error will be active if the temperature is below or above the defined range.
    bool    Interference;         //Interference with another radar sensor has been detected.
    bool    Persistent_Error;     //An internal error which might not disappear after a reset has been detected.
    int     MaxDistanceCfg;       //Current configuration for maximum distance of far scan
    int     SensorID;             //Sensor ID 0 - 7
    int     SortIndex;            //Current configuration of sorting index for object list
    int     RadarPowerCfg;        //Current configuration of transmitted radar power parameter
    bool    CtrlRelayCfg;         //True if relay control message is sent
    int     OutputTypeCfg;        //Currently selected output type as either clusters or objects
    bool    SendQualityCfg;       //True if quality information is sent for clusters or objects
    bool    SendExtInfoCfg;       //True if extended information is sent for objects
    int     MotionRxState;        //Shows the state of the speed and yaw rate input signals
    bool    RCS_Threshold;        //If true, the sensor’s high sensitivity mode is active
};
typedef struct _Radar_State_ARS408_data_t Radar_State_ARS408_data_t;

#ifdef RTMAPS
        const MAPSTypeFilterBase MAPSFilterRadar_State_ARS408_data_t = MAPS_FILTER_USER_STRUCTURE(Radar_State_ARS408_data_t);
#endif

struct _Radar_ARS408_data_t
{
    ObjectStatus_ARS408_data_t status;
    Objects_ARS408_data_t objects[MAX_OBJECTS_408];      // Son 100 objetos como máximo en modo Object
    Radar_State_ARS408_data_t radarState;
};
typedef struct _Radar_ARS408_data_t Radar_ARS408_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterRadar_ARS408_data_t = MAPS_FILTER_USER_STRUCTURE(Radar_ARS408_data_t);
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////// SRR-208-21 ////////////////////////////////////////////////
struct _TrackStatus_data_t
{
    int RadarID;
    int NumOfTracks;            //Number of measured tracks
    int TrackSt_RollCount;      //The rolling counter is incremented with each valid message
    int receivedNumOfTracks1;
    int receivedNumOfTracks2;
    bool    completed;
    // Carlota
    unsigned long long writingTime;
};
typedef struct _TrackStatus_data_t TrackStatus_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterTrackStatus_data_t = MAPS_FILTER_USER_STRUCTURE(TrackStatus_data_t);
#endif

struct _Tracks_data_t
{
    int     Track_ID;           //Unique track ID
    double  Track_LongDispl;    //Longitudinal displacement
    double  Track_LongDispl_Ref408;
    int     Track_Index;        //Current index of the track in the track list
    double  Track_LatDispl;     //Lateral displacement
    double  Track_LatDispl_Ref408;
    double  Track_VrelLong;     //Relative longitudinal velocity (Obstacles which oncoming have got a negative prefix and leaving obstacles are positive.)
    double  Track_VrelLong_Ref408;
    int     Track_Roll_Count;   //The rolling counter is incremented with each valid message
    double  Track_VrelLat;      // Relative lateral velocity (Obstacles which move from left to right have got a negative prefix and from right to left, they are positive.)
    double  Track_VrelLat_Ref408; 
    
    double  Track_RCSValue;//-50 to 30 //Radar cross section (RCS) is the measure of the reflective strength of a cluster
    //or in other words it is the measure of power scattered in a given spatial direction when a cluster is illuminated by an incident wave.
    double  Track_Lifetime;     //The current lifetime of the track. In case the lifetime exceeds 6553.5, the value remains at this maximum value. No overflow takes place.
    int     Track_Index2;       //Current track list index of this track
    int     Track_Roll_Count2;  //The rolling counter is incremented with each valid message
    double  distance; //distancia
    double  angle; //ángulo con respecto al radar
};
typedef struct _Tracks_data_t Tracks_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterTracks_data_t = MAPS_FILTER_USER_STRUCTURE(Tracks_data_t);
#endif

struct _Radar_SRR_data_t
{
    TrackStatus_data_t status;
    Tracks_data_t objects[MAX_OBJECTS_208];      // Son 25 objetos como máximo en modo Track
};
typedef struct _Radar_SRR_data_t Radar_SRR_data_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterRadar_SRR_data_t = MAPS_FILTER_USER_STRUCTURE(Radar_SRR_data_t);
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct _UAH_Single_Obstacle_Structure
{
    int16_t   obstacle_ID;    //Obstacle identifier. Consistent between cycles
    //type {Obstacle_type_pedestrian = 0, Obstacle_type_animal = 1, Obstacle_type_static = 2, Obstacle_type_vehicle = 3, Obstacle_type_bus_stop = 4, Obstacle_type_unknown = 5 };
    uint8_t   type;            //Obstacle type. TBD. e.g.: pedestrian, animal, generic static element, vehicle, bus stop, etc.
    int16_t   long_pos;       //longitudinal position (y) in cm
    int16_t   lat_pos;        //lateral position (x) in cm
    int16_t   long_speed;     //speed longitudinal (y) in cm/s
    int16_t   lat_speed;      //speed lateral (x) in cm/s
    // int16_t   long_pos_camera;       //longitudinal position (y) in cm
    // int16_t   lat_pos_camera;        //lateral position (x) in cm
    // int16_t   long_speed_camera;     //speed longitudinal (y) in cm/s
    // int16_t   lat_speed_camera;      //speed lateral (x) in cm/s
    int16_t   width;          //Bounding box width, in cm
    int16_t   height;         //Bounding box height, in cm
    uint8_t   confidence;     //[0-100] 100: very high confidence. 0 no confidence. ONLY CAMERA = 33, ONLY RADAR = 66, BOTH = 100
    // double  Track_Lifetime;  
    // float   u;       //longitudinal position in px
    // float   v;       //lateral position in px
    // double    angle; 	      //ángulo del objeto con respecto al radar
    // double    distance;      //distancia
    // double    uncertainty;    //incertidumbre asociada al objeto
};
typedef struct _UAH_Single_Obstacle_Structure UAH_Single_Obstacle_Structure_t;

#ifdef RTMAPS
    const MAPSTypeFilterBase MAPSFilterUAH_Single_Obstacle_Structure = MAPS_FILTER_USER_STRUCTURE(UAH_Single_Obstacle_Structure_t);
#endif

struct _UAH_Single_Obstacle_Structure_Camera
{
    /*int16_t   obstacle_ID;    //Obstacle identifier. Consistent between cycles
    //type {Obstacle_type_pedestrian = 0, Obstacle_type_animal = 1, Obstacle_type_static = 2, Obstacle_type_vehicle = 3, Obstacle_type_bus_stop = 4, Obstacle_type_unknown = 5 };
    uint8_t   type;            //Obstacle type. TBD. e.g.: pedestrian, animal, generic static element, vehicle, bus stop, etc.
    int16_t   u;       //longitudinal position in px
    int16_t   v;       //lateral position in px
    int16_t   width;          //Bounding box width, in px
    int16_t   height;         //Bounding box height, in px
    uint8_t   confidence;     //[0-100] 100: very high confidence. 0 no confidence. 
detecciones radar
 	int obj_n;
        int id;
        float x;
        float y;
        float vx;
        float vy;
        float rcs;
        int lt_poe;
        uint64_t t;
        int associated_frame; 
  typedef struct {
        int frame;
        int cloud;
        uint64_t t_0;
        uint64_t t_1;
    } cam_entry_t;
    typedef std::vector<cam_entry_t> CameraLog;
*/
   int frame;
   int id;
   int category; // {Obstacle_type_pedestrian = 1, Obstacle_type_Bicycle = 2, Obstacle_type_car = 3, Obstacle_type_Motorcycle = 4, Obstacle_type_bus = 6, Obstacle_type_Truck = 8 };
   float x0;
   float y0;
   float xf;
   float yf;
   float conf;  //[0-1] 1: very high confidence. 0 no confidence.
};
typedef struct _UAH_Single_Obstacle_Structure_Camera UAH_Single_Obstacle_Structure_Camera_t;


struct _UAH_Obstacles_Structure_408
{
    UAH_Single_Obstacle_Structure_t obstacle[MAX_OBJECTS_408]; //100
    int numOfObstacles;
};
typedef struct _UAH_Obstacles_Structure_408 UAH_Obstacles_Structure_408_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterUAH_Obstacles_Structure_408 = MAPS_FILTER_USER_STRUCTURE(UAH_Obstacles_Structure_408_t);
#endif

struct _UAH_Obstacles_Structure_308
{
    UAH_Single_Obstacle_Structure_t obstacle[MAX_OBJECTS_308];  //40
    int numOfObstacles;
};
typedef struct _UAH_Obstacles_Structure_308 UAH_Obstacles_Structure_308_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterUAH_Obstacles_Structure_308 = MAPS_FILTER_USER_STRUCTURE(UAH_Obstacles_Structure_308_t);
#endif

struct _UAH_Obstacles_Structure_208
{
    UAH_Single_Obstacle_Structure_t obstacle[MAX_OBJECTS_208];  //25
    int numOfObstacles;
};
typedef struct _UAH_Obstacles_Structure_208 UAH_Obstacles_Structure_208_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterUAH_Obstacles_Structure_208 = MAPS_FILTER_USER_STRUCTURE(UAH_Obstacles_Structure_208_t);
#endif

struct _UAH_Obstacles_Structure
{
        uint8_t	  				message_id;
        uint8_t					obstacle_number;
	UAH_Single_Obstacle_Structure_t 	obstacle[20];
};
typedef struct _UAH_Obstacles_Structure UAH_Obstacles_Structure_t;

#ifdef RTMAPS
    const MAPSTypeFilterBase MAPSFilterUAH_Obstacles_Structure = MAPS_FILTER_USER_STRUCTURE(UAH_Obstacles_Structure_t);
#endif

struct _UAH_Obstacles_Structure_Camera
{
        uint8_t	  				message_id;
        uint8_t					obstacle_number;
	UAH_Single_Obstacle_Structure_Camera_t 	obstacle[20];
};
typedef struct _UAH_Obstacles_Structure_Camera UAH_Obstacle_Structure_Camera_t;

#ifdef RTMAPS
	const MAPSTypeFilterBase MAPSFilterUAH_Obstacles_Structure_Camera = MAPS_FILTER_USER_STRUCTURE(UAH_Obstacles_Structure_Camera_t);
#endif


struct _UAH_Single_Detection_Structure
{
    int     detection_ID;
    double  px;
    double  py;
    double  polar_ang;
    double  polar_dist;
    double  perpendicular_error;
    double  longitudinal_error;
    int     Obj_ProbOfExist;
    double  Track_Lifetime;
    int     zone;
};
typedef struct _UAH_Single_Detection_Structure UAH_Single_Detection_Structure_t;

#ifdef RTMAPS
    const MAPSTypeFilterBase MAPSFilterUAH_Single_Detection_Structure = MAPS_FILTER_USER_STRUCTURE(UAH_Single_Detection_Structure_t);
#endif

struct _UAH_Detections_Structure
{
    uint8_t                             num_detections;
    UAH_Single_Detection_Structure_t    detection[MAX_OBJECTS_208+MAX_OBJECTS_208+MAX_OBJECTS_408];
};
typedef struct _UAH_Detections_Structure UAH_Detections_Structure_t;

#ifdef RTMAPS
    const MAPSTypeFilterBase MAPSFilterUAH_Detections_Structure = MAPS_FILTER_USER_STRUCTURE(UAH_Detections_Structure_t);
#endif

//Now re-enable structure alignment
#pragma pack(pop)

struct _BRAVE_Single_Detection_Structure
{
    int     id;
    double  longitudinal;
    double  lateral;
    double  velocidad_lon;
    double  reproj_x;
    double  reproj_y;
    double  UTC_timeMS; //utc_ms+delta_writing_time
    int     zone;
};
typedef struct _BRAVE_Single_Detection_Structure BRAVE_Single_Detection_Structure_t;


struct _BRAVE_Detections_Structure
{
    int                                   num_detections;
    BRAVE_Single_Detection_Structure_t    detection[MAX_OBJECTS_208+MAX_OBJECTS_208+MAX_OBJECTS_408];
};
typedef struct _BRAVE_Detections_Structure BRAVE_Detections_Structure_t;


struct _BRAVE_Single_Sending_Structure
{
    int     id;
    double  longitudinal;
    double  lateral;
    double  velocidad_lon;
    double  reproj_x;
    double  reproj_y;
    double  UTC_timeMS; //utc_ms+delta_writing_time
};
typedef struct _BRAVE_Single_Sending_Structure BRAVE_Single_Sending_Structure_t;

struct _BRAVE_Sending_Structure
{
    int                                   num_detections;
    BRAVE_Single_Sending_Structure_t      detection[30];//10
};
typedef struct _BRAVE_Sending_Structure BRAVE_Sending_Structure_t;

#endif
