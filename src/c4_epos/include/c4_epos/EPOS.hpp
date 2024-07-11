#ifndef EPOS_H
#define EPOS_H

#include "c4_epos/Definitions.h"

#include "stdlib.h"
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <stdio.h>

enum eControlMode_t {posicion = 1, velocidad = 2};

class EPOS
{
private:
  bool m_oInitialisation;
  bool m_oImmediately;
  bool m_oUpdateActive;  
  void * m_KeyHandle;
  unsigned short m_wNodeId;
  unsigned int m_dErrorCode;
  char m_uMode;
  int m_lStartPosition;
  int m_lTargetPosition;
  int m_lTargetSpeed;
  unsigned int m_dProfileVelocity;
  unsigned int m_dProfileAcceleration;
  unsigned int m_dProfileDeceleration;
  int Status;
  unsigned int ProfileVelocity;
  unsigned int ProfileAcceleration;
  unsigned int ProfileDeceleration;
  short unsigned int Kp, Ki, Kd;
  unsigned short int m_estado_outputs;
  double m_Kp, m_Ki, m_Kd;
  eControlMode_t m_tipo_inicializacion_EPOS;
  short int m_Current;

public:
  EPOS(int node_id, bool USB_invertido, double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, eControlMode_t tipo_inicializacion_EPOS = posicion);
  bool OpenDevice(bool USB_invertido = false);

  void SetAnalogOutput(unsigned short value);
  void GetAnalogInput(unsigned short *value, unsigned short port);
  void GetAllDigitalInput(unsigned short *value);
  bool GetDigitalInput(unsigned short *value, unsigned short port);
  void SetDigitalOutput(bool value, unsigned short port);
  void Enable();
  void Disable();
  void Halt();
  void setHome(int homePosition);
  void getCurrent();
  void Move(signed long Valor);
  void MoveRelative(signed long Valor);
  void MoveAbsolute(signed long Valor);
  void MoveVelocity(signed long velocity);
  int GetVelocityAveraged();
  int GetVelocity();
  int GetStatus();
  int GetState();
  int GetFaultState();
  ~EPOS();
  //void SetHome(long Home);
  signed long GetPos();
  short unsigned int GetKp();
  short unsigned int GetKi();
  short unsigned int GetKd();
  signed short int GetCurrentIs();
};


#endif
