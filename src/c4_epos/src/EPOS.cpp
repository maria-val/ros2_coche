#include "c4_epos/EPOS.hpp"

/************************************************************************
Constructor de la clase
************************************************************************/
EPOS::EPOS(int node_id, bool USB_invertido, double Kp, double Ki, double Kd, eControlMode_t tipo_inicializacion_EPOS){
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
    m_tipo_inicializacion_EPOS = tipo_inicializacion_EPOS;
    m_oImmediately = true;
    m_wNodeId = node_id;
    m_estado_outputs = 0;

    if (node_id==0){
        ProfileVelocity = 15800;//5000//15800; esta es la maxima para el volante
        ProfileAcceleration = 100000; //50000;//100000;
        ProfileDeceleration = 100000; //50000;//100000;
    }else if (node_id==1){
        ProfileVelocity = 15000;//15800;//5000//15800; esta es la maxima para el volante
        ProfileAcceleration = 20000;//50000; //50000;//100000;
        ProfileDeceleration = 20000;//50000; //50000;//100000;
    }else{ //node_id == 2 ---> EPOS FRENO
        ProfileVelocity = 2000;//5000;
        ProfileAcceleration = 5000;//15000;
        ProfileDeceleration = 5000;//15000;
    }
    std::cerr << "Paso 1.1" << std::endl;
    if(OpenDevice(USB_invertido))
    {
        std::cerr << "Paso 1 if" << std::endl;
        m_oInitialisation = true;
        m_oUpdateActive = true;
        Status = 1;
    }
    else
    {
        std::cerr << "Paso 1 else" << std::endl;
        m_oInitialisation = false;
        m_oUpdateActive = false;
        Status = 0;
    }
    std::cerr << "Paso 1.2" << std::endl;
}

/*
Position Profile Mode   1 OMD_PROFILE_POSITION_MODE
Position Velocity Mode  3 OMD_PROFILE_VELOCITY_MODE
Homing Mode             6 OMD_HOMING_MODE
Interpolated Position Mode 7 OMD_INTERPOLATED_POSITION_MODE
Position Mode           -1 OMD_POSITION_MODE
Velocity Mode           -2 OMD_VELOCITY_MODE
Current Mode            -3 OMD_CURRENT_MODE
Master Encoder Mode     -5 OMD_MASTER_ENCODER_MODE
Step Direction Mode     -6 OMD_STEP_DIRECTION_MODE
*/

/************************************************************************
Opens communication interface via the OpenDeviceDlg
************************************************************************/
bool EPOS::OpenDevice(bool USB_invertido)
{
    char *pStrProtocolStackName;

    char* pDeviceName = new char[255];
    char* pProtocolStackName = new char[255];
    char* pInterfaceName = new char[255];
    char* pPortName = new char[255];
    std::cerr << "Paso 4" << std::endl;
    //Allocate memory for string
    if((pStrProtocolStackName = (char*)malloc(100)) == NULL)
    {
        std::cerr << "Paso 5" << std::endl;
        //Application->MessageBoxA("Not enough memory to allocate buffer\n","System",MB_OK);
        std::cerr << "Not enough memory to allocate buffer" << std::endl;
        exit(1);   // Exit the program, if not enough memory
    }
    std::cerr << "Paso 6" << std::endl;
    //m_KeyHandle = VCS_OpenDeviceDlg(&m_dErrorCode);
    if (m_wNodeId==1){
        std::cerr << "Paso 6 if" << std::endl;
        strcpy(pDeviceName, "EPOS2");
        strcpy(pProtocolStackName, "EPOS2_USB");
        strcpy(pInterfaceName, "USB");
        //  strcpy(pPortName, "maxv");
        if (USB_invertido)
            strcpy(pPortName, "USB1");
        else
            strcpy(pPortName, "USB0");
        std::cerr << "Paso 6 opendev" << std::endl;
        m_KeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &m_dErrorCode);
        std::cerr << m_KeyHandle << std::endl;
        //m_KeyHandle = VCS_OpenDevice("EPOS2", "EPOS2_USB", "USB", "USB0", &m_dErrorCode);
        std::cerr << "Paso 6 if end" << std::endl;

    }
    else
    {
        std::cerr << "Paso 6 else" << std::endl;
        //strcpy(pDeviceName, "EPOS2");
        int endSelect;
        VCS_GetDeviceNameSelection(true, pDeviceName, 255, &endSelect, &m_dErrorCode);
        std::cerr << pDeviceName << std::endl;
        //strcpy(pProtocolStackName, "EPOS2_USB");
        VCS_GetProtocolStackNameSelection(pDeviceName, true, pProtocolStackName, 255, &endSelect, &m_dErrorCode);
        std::cerr << pProtocolStackName << std::endl;
        //strcpy(pInterfaceName, "USB");
        VCS_GetInterfaceNameSelection(pDeviceName, pProtocolStackName, true, pInterfaceName, 255, &endSelect, &m_dErrorCode);
        std::cerr << pInterfaceName << std::endl;
        //      strcpy(pPortName, "maxf");
        if (USB_invertido)
            strcpy(pPortName, "USB0");
        else
          /*{
            VCS_GetPortNameSelection(pDeviceName, pProtocolStackName, pInterfaceName, true, pPortName, 255, &endSelect, &m_dErrorCode);
            std::cerr << pPortName << std::endl;
          }*/
            strcpy(pPortName, "USB1");

        m_KeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, &m_dErrorCode);
        //m_KeyHandle = VCS_OpenDevice("EPOS2", "MAXON SERIAL V2", "USB", "USB1", &m_dErrorCode);
        std::cerr << m_KeyHandle << std::endl;
    }

    int usleep_time = 1000; //ATENCIÓN: Cada acceso a la EPOS tarda 4ms. No hemos visto que se necesite un sleep para que se realizen
    // correctamente las llamadas. De todas maneras vamos a mantener estos sleep despues de cada llamada a la EPOS.
    usleep(usleep_time);

    //OJO CON EL BAUDRATE WARNING RUBEN
    VCS_SetProtocolStackSettings(m_KeyHandle, 1000000, 500, &m_dErrorCode);
    usleep(usleep_time);
    std::cerr << "Paso 7" << std::endl;
    if(m_KeyHandle)
    {
        //Clear Error History
        if(VCS_ClearFault(m_KeyHandle,m_wNodeId,&m_dErrorCode))
        {
            usleep(usleep_time);
            //Read Operation Mode
            if(VCS_GetOperationMode(m_KeyHandle,m_wNodeId,&m_uMode,&m_dErrorCode))
            {
                usleep(usleep_time);
                //Write Operational Mode (Profile Position Mode)
                if (m_wNodeId==1) //VOLANTE
                {
                    bool control_en_velocidad = false;
                    if (m_tipo_inicializacion_EPOS == 1) control_en_velocidad = false;
                    else if (m_tipo_inicializacion_EPOS == 2) control_en_velocidad = true;
                    else
                    {
                        std::cerr << "No se ha podido inicializar la EPOS del VOLANTE" << std::endl;
                        free(pStrProtocolStackName);
                        return false;
                    }

                    if (control_en_velocidad)
                    {
                        if(VCS_SetOperationMode(m_KeyHandle,m_wNodeId, OMD_PROFILE_VELOCITY_MODE,&m_dErrorCode)) //control en velocidad
                        {
                            usleep(usleep_time);

                            VCS_GetOperationMode(m_KeyHandle,m_wNodeId,&m_uMode,&m_dErrorCode);
                            usleep(usleep_time);
                            if (m_uMode != OMD_PROFILE_VELOCITY_MODE)
                                std::cerr << "VOLANTE ERROR VCS_GetOperationMode" <<  std::endl;

                            //Write Velocity Profile Objects
                            if(VCS_SetVelocityProfile(m_KeyHandle,m_wNodeId,ProfileAcceleration,ProfileDeceleration,&m_dErrorCode))
                            {
                                usleep(usleep_time);
                                std::cerr << "VCS_SetVelocityProfile Acceleration " << ProfileAcceleration << " Deceleration " << ProfileDeceleration <<  std::endl;

                                //Read Actual Position
                                if(VCS_GetPositionIs(m_KeyHandle,m_wNodeId,&m_lStartPosition,&m_dErrorCode))
                                {
                                    usleep(usleep_time);
                                    std::cerr << "GetPosition " << m_lStartPosition <<  std::endl;

                                    int estado;
                                    VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                    usleep(usleep_time);
                                    while (estado == 0)
                                    {
                                        VCS_SetEnableState(m_KeyHandle, m_wNodeId, &m_dErrorCode);
                                        usleep(usleep_time);
                                        VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                        usleep(usleep_time);
                                    }
                                    VCS_GetVelocityRegulatorGain(m_KeyHandle, m_wNodeId, &Kp, &Ki, &m_dErrorCode);
                                    usleep(usleep_time);
                                    std::cerr << "VOLANTE GetVelocityRegulatorGain Kp " << Kp << " Ki " << Ki <<  std::endl;

                                    //VCS_SetVelocityRegulatorGain(m_KeyHandle, m_wNodeId, m_Kp, m_Ki, &m_dErrorCode);
                                    //std::cerr << "VOLANTE SetVelocityRegulatorGain Kp " << m_Kp << " Ki " << m_Ki <<  std::endl;
                                    //usleep(usleep_time);
                                    return true;
                                }
                            }
                        }
                    }
                    else
                    {
                        if(VCS_SetOperationMode(m_KeyHandle, m_wNodeId, OMD_PROFILE_POSITION_MODE, &m_dErrorCode)) //control en posicion OMD_POSITION_MODE WARNING RUBEN
                        {
                            usleep(usleep_time);

                            unsigned short nomCurrent, maxCurrent, thermalK;
                            VCS_GetDcMotorParameter(m_KeyHandle, m_wNodeId, &nomCurrent, &maxCurrent, &thermalK, &m_dErrorCode);
                            usleep(usleep_time);
                            std::cerr << "GET VOLANTE nomcurrent " << nomCurrent << " maxcurrent " << maxCurrent << " thermalK " << thermalK <<  std::endl;
                            VCS_SetDcMotorParameter(m_KeyHandle, m_wNodeId, 3000, 10000, thermalK, &m_dErrorCode);
                            usleep(usleep_time);
                            VCS_GetDcMotorParameter(m_KeyHandle, m_wNodeId, &nomCurrent, &maxCurrent, &thermalK, &m_dErrorCode);
                            usleep(usleep_time);
                            std::cerr << "GET - AFTER SET - VOLANTE nomcurrent " << nomCurrent << " maxcurrent " << maxCurrent << " thermalK " << thermalK <<  std::endl;




                            unsigned int maximalError;
                            VCS_GetMaxFollowingError(m_KeyHandle, m_wNodeId, &maximalError, &m_dErrorCode);
                            std::cerr << "VOLANTE maximal error " << maximalError <<  std::endl;


                            VCS_GetOperationMode(m_KeyHandle,m_wNodeId,&m_uMode,&m_dErrorCode);
                            usleep(usleep_time);
                            if (m_uMode != OMD_PROFILE_POSITION_MODE)
                                std::cerr << "VOLANTE ERROR VCS_GetOperationMode" <<  std::endl;
                            if(VCS_SetPositionProfile(m_KeyHandle,m_wNodeId,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&m_dErrorCode))
                            {
                                usleep(usleep_time);
                                std::cerr << "VCS_SetPositionProfile Velocity " << ProfileVelocity << " Acceleration " << ProfileAcceleration << " Deceleration " << ProfileDeceleration <<  std::endl;

                                //Read Actual Position
                                if(VCS_GetPositionIs(m_KeyHandle,m_wNodeId,&m_lStartPosition,&m_dErrorCode))
                                {
                                    usleep(usleep_time);
                                    std::cerr << "GetPosition " << m_lStartPosition <<  std::endl;

                                    int estado;
                                    VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                    usleep(usleep_time);
                                    while (estado == 0){
                                        VCS_SetEnableState(m_KeyHandle, m_wNodeId, &m_dErrorCode);
                                        usleep(usleep_time);
                                        VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                        usleep(usleep_time);
                                    }

                                    VCS_GetPositionRegulatorGain(m_KeyHandle, m_wNodeId, &Kp, &Ki, &Kd, &m_dErrorCode);
                                    usleep(usleep_time);
                                    std::cerr << "VCS_GetPositionRegulatorGain VOLANTE Kp " << Kp << " Ki " << Ki << " Kd " << Kd <<  std::endl;
                                    //VCS_GetPositionRegulatorGain Kp 586 Ki 2093 Kd 842
                                    //VCS_SetPositionRegulatorGain(m_KeyHandle, m_wNodeId, 586, 2093, 842, &m_dErrorCode);
                                    //std::cerr << "VOLANTE VCS_SetPositionRegulatorGain Kp " << 586 << " Ki " << 2093 << " Kd " << 842 << std::endl;
                                    usleep(usleep_time);
                                    return true;
                                }
                            }
                        }
                    }
                }
                else //FRENO
                {
                    //Control en posicion
                    if(VCS_SetOperationMode(m_KeyHandle,m_wNodeId, OMD_PROFILE_POSITION_MODE,&m_dErrorCode)) //control en posicion
                    {
                        usleep(usleep_time);

                        unsigned short nomCurrent, maxCurrent, thermalK;
                        VCS_GetDcMotorParameter(m_KeyHandle, m_wNodeId, &nomCurrent, &maxCurrent, &thermalK, &m_dErrorCode);
                        usleep(usleep_time);
                        std::cerr << "GET FRENO nomcurrent " << nomCurrent << " maxcurrent " << maxCurrent << " thermalK " << thermalK <<  std::endl;
                        VCS_SetDcMotorParameter(m_KeyHandle, m_wNodeId, 3000, 4000, thermalK, &m_dErrorCode);
                        usleep(usleep_time);
                        VCS_GetDcMotorParameter(m_KeyHandle, m_wNodeId, &nomCurrent, &maxCurrent, &thermalK, &m_dErrorCode);
                        usleep(usleep_time);
                        std::cerr << "GET - AFTER SET - FRENO nomcurrent " << nomCurrent << " maxcurrent " << maxCurrent << " thermalK " << thermalK <<  std::endl;

                        unsigned int maximalError;
                        VCS_GetMaxFollowingError(m_KeyHandle, m_wNodeId, &maximalError, &m_dErrorCode);
                        std::cerr << "FRENO maximal error " << maximalError <<  std::endl;


                        VCS_GetOperationMode(m_KeyHandle,m_wNodeId,&m_uMode,&m_dErrorCode);
                        usleep(usleep_time);
                        if (m_uMode != OMD_PROFILE_POSITION_MODE)
                            std::cerr << "FRENO ERROR VCS_GetOperationMode" <<  std::endl;

                        //Write Position Profile Objects
                        if(VCS_SetPositionProfile(m_KeyHandle,m_wNodeId,ProfileVelocity,ProfileAcceleration,ProfileDeceleration,&m_dErrorCode))
                        {
                            usleep(usleep_time);
                            std::cerr << "SetPositionProfile velocity " << ProfileVelocity << " Acceleration " << ProfileAcceleration << " Deceleration " << ProfileDeceleration <<  std::endl;

                            //Read Actual Position
                            if(VCS_GetPositionIs(m_KeyHandle,m_wNodeId,&m_lStartPosition,&m_dErrorCode))
                            {
                                usleep(usleep_time);
                                std::cerr << "GetPosition " << m_lStartPosition <<  std::endl;

                                int estado;
                                VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                usleep(usleep_time);
                                while (estado == 0){
                                    VCS_SetEnableState(m_KeyHandle, m_wNodeId, &m_dErrorCode);
                                    usleep(usleep_time);
                                    VCS_GetEnableState(m_KeyHandle, m_wNodeId, &estado, &m_dErrorCode);
                                    usleep(usleep_time);
                                    std::cout << " EPOS FRENO ESTADO  == 0" << std::endl;
                                }
                                VCS_GetPositionRegulatorGain(m_KeyHandle, m_wNodeId, &Kp, &Ki, &Kd, &m_dErrorCode);
                                usleep(usleep_time);
                                std::cerr << "GetPositionRegulatorGain FRENO Kp " << Kp << " Ki " << Ki << " Kd " << Kd <<  std::endl;
                                return true;
                            }
                        }
                    }
                }
            }
        }
        // ShowErrorInformation(m_dErrorCode);
    }
    else
    {
        std::cerr << "Paso 8" << std::endl;
        if (m_wNodeId==1)
            std::cerr << "No se ha podido inicializar la EPOS del VOLANTE" << std::endl;
        else if (m_wNodeId==2)
            std::cerr << "No se ha podido inicializar la EPOS del FRENO" << std::endl;

        free(pStrProtocolStackName);
    }
    std::cerr << "Paso 9" << std::endl;
    return false;
}

//------------------------------------------------------------------------------------

void  EPOS::Enable()
{
    // bool oFault = false;



    /*    if(!VCS_GetFaultState(m_KeyHandle,m_wNodeId,&oFault,&m_dErrorCode))
    {
     //	ShowErrorInformation(m_dErrorCode);
        return;
    }

    if(oFault)
    {
        if(!VCS_ClearFault(m_KeyHandle,m_wNodeId,&m_dErrorCode))
        {
           // ShowErrorInformation(m_dErrorCode);
            return;
        }
    }    */

    if(!VCS_SetEnableState(m_KeyHandle,m_wNodeId,&m_dErrorCode))
    {
        //ShowErrorInformation(m_dErrorCode);
    }
}


//------------------------------------------------------------------------------------
/************************************************************************
Sets device to disable state
************************************************************************/
void EPOS::Disable()
{
    if(!VCS_SetDisableState(m_KeyHandle,m_wNodeId,&m_dErrorCode))
    {
        //Application->MessageBoxA("Communication error","Error",MB_OK);
    }
}
//---------------------------------------------------------------------------

/************************************************************************
Stops the movement
************************************************************************/
void EPOS::Halt()
{
    if (m_uMode != OMD_PROFILE_POSITION_MODE)
    {
        if(!VCS_HaltPositionMovement(m_KeyHandle,m_wNodeId,&m_dErrorCode))
        {
            // Application->MessageBoxA("Communication error","Error",MB_OK);
        }
    }
    else if (m_uMode != OMD_PROFILE_VELOCITY_MODE)
    {
        if(!VCS_HaltVelocityMovement(m_KeyHandle, m_wNodeId, &m_dErrorCode))
        {
            // Application->MessageBoxA("Communication error","Error",MB_OK);
        }
    }
}
//---------------------------------------------------------------------------

/************************************************************************
Starts the movement
************************************************************************/

void EPOS::setHome(int homePosition)
{
    VCS_DefinePosition(m_KeyHandle,m_wNodeId, homePosition, &m_dErrorCode);
}

void EPOS::Move(signed long Valor)
{

    m_lTargetPosition = Valor;

    if (m_wNodeId == 1) // volante
    {
        MoveRelative(Valor);
    }
    else if (m_wNodeId == 2) // freno
    {
        MoveAbsolute(Valor);
    }
}

void EPOS::MoveRelative(signed long Valor)
{
    static signed long lastCommand=0;
    m_lTargetPosition = Valor;
    if(abs(lastCommand - m_lTargetPosition) > 100)
    {
        if(abs(lastCommand - m_lTargetPosition) > 300000)
        {
            std::cerr << "ERROR MOVIMIENTO DE MAS DE 300000 CUENTAS EN EPOS " << m_wNodeId << std::endl;
        }
        else
        {
            VCS_MoveToPosition(m_KeyHandle,m_wNodeId,m_lTargetPosition,/*relativo*/false,m_oImmediately,&m_dErrorCode);
            usleep(5000);
            lastCommand = m_lTargetPosition;
        }
    }
}

void EPOS::MoveAbsolute(signed long Valor){
    static signed long lastCommand=0;
    m_lTargetPosition = Valor;

    if(abs(lastCommand - m_lTargetPosition) > 100)
    {
        if(abs(lastCommand - m_lTargetPosition) > 300000)
        {
            std::cerr << "ERROR MOVIMIENTO DE MAS DE 300000 CUENTAS EN EPOS " << m_wNodeId << " valor a aplicar "  << lastCommand - m_lTargetPosition << std::endl;
        }
        else
        {
            VCS_MoveToPosition(m_KeyHandle,m_wNodeId,m_lTargetPosition,/*absoluto*/true,m_oImmediately,&m_dErrorCode);
            usleep(5000);
            lastCommand = m_lTargetPosition;

            if(m_dErrorCode != 0)
                std::cerr << "Error code EPOS Move Absolute " << m_dErrorCode << std::endl;
        }
    }
}

//---------------------------------------------------------------------------

void EPOS::MoveVelocity(signed long velocity)
{
    m_lTargetSpeed = velocity;
    if (m_wNodeId == 1)
    {
        if(!VCS_MoveWithVelocity(m_KeyHandle,m_wNodeId,m_lTargetSpeed,&m_dErrorCode))
        {
            std::cout << m_dErrorCode << " " << m_lTargetSpeed << " " << m_wNodeId << std::endl;
        }
    }
}

/************************************************************************
Destructor de la clase
************************************************************************/

EPOS::~EPOS()
{
    //Disable();
    //VCS_CloseAllDevices(&m_dErrorCode);
    std::cout << "Destructor EPOS " <<  GetStatus() << std::endl;
    Halt();

    if (GetStatus())
        VCS_CloseDevice(m_KeyHandle, &m_dErrorCode);
    usleep(1000);
}
//---------------------------------------------------------------------------

/************************************************************************
Estado del controlador
************************************************************************/

int EPOS::GetStatus()
{
    return Status;
}

int EPOS::GetState()
{   // EPOS COMMAND LIBRARY TABLAS 5-14
    unsigned short state;
    VCS_GetState(m_KeyHandle, m_wNodeId, &state, &m_dErrorCode);
    std::cerr << "EPOS " << m_wNodeId << " state " << state << std::endl;


    unsigned short word1, pNominalCurrent, pMaxOutputCurrent, pThermalTimeConstant, pSensorType, pp, ip, dp, pc, ic;
    unsigned int pEncoderResolution;
    int pInvertedPolarity;

    VCS_SetCurrentRegulatorGain(m_KeyHandle, m_wNodeId, 100, 0, &m_dErrorCode);
    VCS_SetPositionRegulatorGain(m_KeyHandle, m_wNodeId, 100, 100, 0, &m_dErrorCode);



    VCS_GetMotorType(m_KeyHandle, m_wNodeId, &word1, &m_dErrorCode);
    std::cout << "motor type " <<word1 << std::endl;
    VCS_GetDcMotorParameter(m_KeyHandle, m_wNodeId, &pNominalCurrent, &pMaxOutputCurrent, &pThermalTimeConstant, &m_dErrorCode);
    std::cout << "pNominalCurrent " <<pNominalCurrent << std::endl;
    std::cout << "pMaxOutputCurrent " <<pMaxOutputCurrent << std::endl;
    std::cout << "pThermalTimeConstant " <<pThermalTimeConstant << std::endl;
    VCS_GetSensorType(m_KeyHandle, m_wNodeId, &pSensorType, &m_dErrorCode);
    std::cout << "pSensorType " <<pSensorType << std::endl;
    VCS_GetIncEncoderParameter(m_KeyHandle, m_wNodeId, &pEncoderResolution, &pInvertedPolarity, &m_dErrorCode);
    std::cout << "pEncoderResolution " <<pEncoderResolution << std::endl;
    std::cout << "pInvertedPolarity " <<pInvertedPolarity << std::endl;
    VCS_GetPositionRegulatorGain(m_KeyHandle,  m_wNodeId, &pp, &ip, &dp, &m_dErrorCode);
    std::cout << "pp " <<pp << std::endl;
    std::cout << "ip " <<ip << std::endl;
    std::cout << "dp " <<dp << std::endl;
    VCS_GetCurrentRegulatorGain(m_KeyHandle, m_wNodeId, &pc, &ic, &m_dErrorCode);
    std::cout << "pc " <<pc << std::endl;
    std::cout << "ic " <<ic << std::endl;

    return state;
}

int EPOS::GetFaultState()
{
    unsigned char nerrors;
    VCS_GetNbOfDeviceError(m_KeyHandle, m_wNodeId, &nerrors, &m_dErrorCode);


    for(char i = 1; i <= nerrors; i++)
    {
        unsigned int error_code;
        VCS_GetDeviceErrorCode(m_KeyHandle, m_wNodeId, i, &error_code, &m_dErrorCode);
        std::cerr << "Error code " << (int)i << " " << error_code << std::endl;

        //65290 == FF0A == Position sensor breach error
    }

    return nerrors;
}

//---------------------------------------------------------------------------

void EPOS::SetAnalogOutput(unsigned short value)
{
    static unsigned short lastCommand =0;

    if(lastCommand != value)
    {
        VCS_SetAnalogOutput(m_KeyHandle, m_wNodeId, 0, value, &m_dErrorCode);
        lastCommand = value;
        usleep(5000);
    }
    //std::cout  << "Código de error  " << m_dErrorCode << "   valor  " << value << std::endl;
}

void EPOS::GetAnalogInput(unsigned short *value, unsigned short port)
{

    VCS_GetAnalogInput(m_KeyHandle, m_wNodeId, port, value, &m_dErrorCode);

    //std::cout  << "Código de error  " << m_dErrorCode << "   valor  " << *value << std::endl;
}

void EPOS::GetAllDigitalInput(unsigned short *value)
{
    VCS_GetAllDigitalInputs(m_KeyHandle, m_wNodeId, value, &m_dErrorCode);

    //std::cout  << "Código de error  " << m_dErrorCode << " Leyendo DI  valor  " << *value << std::endl;
}

bool EPOS::GetDigitalInput(unsigned short *value, unsigned short port)
{
    bool ret = false;
    if(port > 15)
    {
        std::cerr  << "Error el valor de port debe ser entre 0 y 15" << std::endl;
        *value = -1;
    }
    else
    {
        unsigned short mask = 1 << port;

        VCS_GetAllDigitalInputs(m_KeyHandle, m_wNodeId, value, &m_dErrorCode);

        *value &= mask;

        ret = *value & mask;
    }

    return ret;
    std::cout  << "Código de error  " << m_dErrorCode << " Leyendo DI  valor  " << *value << std::endl;
}

void EPOS::SetDigitalOutput(bool value, unsigned short port)
{
    static bool inicializar = true;
    // VCS_DigitalOutputConfiguration(void* KeyHandle, unsigned short NodeId, unsigned short DigitalOutputNb, unsigned short Configuration, int State, int Mask, int Polarity, unsigned int* pErrorCode);

    if(inicializar)
    {
        VCS_DigitalOutputConfiguration(m_KeyHandle, m_wNodeId, 0x01, 15, 0, 1, 0, &m_dErrorCode);
        usleep(5000);
        //std::cout  << "Código de error  " << m_dErrorCode << " Escrito  " << value << std::endl;

        VCS_DigitalOutputConfiguration(m_KeyHandle, m_wNodeId, 0x02, 14, 0, 1, 0, &m_dErrorCode);
        usleep(5000);
        //std::cout  << "Código de error  " << m_dErrorCode << " Escrito  " << value << std::endl;

        VCS_SetAllDigitalOutputs(m_KeyHandle, m_wNodeId, 0x00, &m_dErrorCode);
        usleep(5000);
        m_estado_outputs = 0x00;

        inicializar = false;
    }

    //VCS_DigitalOutputConfiguration(m_KeyHandle, m_wNodeId, 0x01, 15, 1, 1, 0, &m_dErrorCode);
    //    std::cout  << "Código de error  " << m_dErrorCode << " Escrito  " << value << std::endl;

    unsigned short mask = 1 << port;
    //std::cerr << "MASK: " << mask << " port: " << port << std::endl;

    if(value)
    {
        m_estado_outputs = m_estado_outputs | mask;
        //std::cerr << "value: " << value << std::endl;
    }
    else
    {
        m_estado_outputs = m_estado_outputs & ~mask;
        //  std::cerr << "value: " << value << std::endl;
    }

    //std::cerr << "Estado_outputs: " << m_estado_outputs << std::endl;
    VCS_SetAllDigitalOutputs(m_KeyHandle, m_wNodeId, m_estado_outputs, &m_dErrorCode);
    usleep(5000);
    // std::cout  << "Código de error  " << m_dErrorCode << " Escrito  " << value << std::endl;
}

/************************************************************************
Obtener el ángulo real del motor
************************************************************************/
//ATENCIÓN: Cualquier llamada a la EPOS tarda 4ms. TENER CUIDADO.
signed long EPOS::GetPos()
{
    int a = VCS_GetPositionIs(m_KeyHandle,m_wNodeId,&m_lStartPosition,&m_dErrorCode);
    //std::cout << "ret " << a << " errorCode " << m_dErrorCode << std::endl;

    return m_lStartPosition;
}

short unsigned int EPOS::GetKp()
{
    return Kp;
}
short unsigned int EPOS::GetKi()
{
    return Ki;
}
short unsigned int EPOS::GetKd()
{
    return Kd;
}

//ATENCIÓN: Cualquier llamada a la EPOS tarda 4ms. TENER CUIDADO.
signed short int EPOS::GetCurrentIs()
{
    VCS_GetCurrentIs(m_KeyHandle,m_wNodeId, &m_Current,&m_dErrorCode);
    return m_Current;
}

//ATENCIÓN: Cualquier llamada a la EPOS tarda 4ms. TENER CUIDADO.
int EPOS::GetVelocity()
{
    int velocity=0;
    if(!VCS_GetVelocityIs(m_KeyHandle,m_wNodeId, &velocity,&m_dErrorCode))
    {
        // Application->MessageBoxA("Communication error","Error",MB_OK);
    }

    return velocity;
}

//ATENCIÓN: Cualquier llamada a la EPOS tarda 4ms. TENER CUIDADO.
int EPOS::GetVelocityAveraged()
{
    int velocity=0;
    if(!VCS_GetVelocityIsAveraged(m_KeyHandle,m_wNodeId, &velocity,&m_dErrorCode))
    {
        // Application->MessageBoxA("Communication error","Error",MB_OK);
    }

    return velocity;
}
