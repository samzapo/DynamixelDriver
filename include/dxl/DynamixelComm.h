//
//    DynamixelComm.h		Class to communicate with USB2Dynamixel
//
//    Copyright (C) 2010  Christian Balkenius
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    See http://www.ikaros-project.org/ for more information.
//
//
//    Created: April 4, 2010
//

#ifndef DYNAMIXELCOMM
#define DYNAMIXELCOMM

#include <cstdlib>
#include <cstdio>
#include <sstream>
#include <vector>

class SerialException
{
public:
    const char *    string;
    int             internal_reference;

    SerialException(const char * s, int ref = 0) : string(s), internal_reference(ref) {};
};

class Serial
{
public:

    Serial();
    Serial(const char * device, unsigned long baud_rate);   // defaults to 8 bits, no partity, 1 stop bit
    ~Serial();

    int SendString(const char *sendbuf);
    int SendBytes(const char *sendbuf, int length);

    int ReceiveUntil(char *rcvbuf, char c);
    int ReceiveBytes(char *rcvbuf, int length);

    void Close();

    void Flush();
    void FlushOut();
    void FlushIn();

private:
    int fd;
};

//--- Control Table Address ---

enum ControlTableRX24F{
 P_MODEL_NUMBER_L    = 0 ,
 P_MODOEL_NUMBER_H   = 1 ,
 P_VERSION           = 2 ,
 P_ID                = 3 ,
 P_BAUD_RATE         = 4 ,
 P_RETURN_DELAY_TIME = 5 ,
 P_CW_ANGLE_LIMIT_L  = 6 ,
 P_CW_ANGLE_LIMIT_H  = 7 ,
 P_CCW_ANGLE_LIMIT_L = 8 ,
 P_CCW_ANGLE_LIMIT_H = 9 ,
 P_SYSTEM_DATA2      = 10,
 P_LIMIT_TEMPERATURE = 11,
 P_DOWN_LIMIT_VOLTAGE= 12,
 P_UP_LIMIT_VOLTAGE  = 13,
 P_MAX_TORQUE_L      = 14,
 P_MAX_TORQUE_H      = 15,    // Max Torque (offline)
 P_RETURN_LEVEL      = 16,
 P_ALARM_LED         = 17,
 P_ALARM_SHUTDOWN    = 18,
 P_OPERATING_MODE    = 19,
 P_KP                = 20,    // These names seem to suggest some form of PID controller
 P_KD                = 21,    // but the AX-12 manual calls address 20-23 up/down calibration
 P_KI                = 22,    // to compensate for potentiometer inaccuracies
 P_IDAMP             = 23,

 P_TORQUE_ENABLE         =24,
 P_LED                   =25,
 P_CW_COMPLIANCE_MARGIN  =26,
 P_CCW_COMPLIANCE_MARGIN =27,
 P_CW_COMPLIANCE_SLOPE   =28,
 P_CCW_COMPLIANCE_SLOPE  =29,
 P_GOAL_POSITION_L       =30,
 P_GOAL_POSITION_H       =31,
 P_GOAL_SPEED_L          =32,
 P_GOAL_SPEED_H          =33,
 P_TORQUE_LIMIT_L        =34,   // Max Torque (online)
 P_TORQUE_LIMIT_H        =35,
 P_PRESENT_POSITION_L    =36,
 P_PRESENT_POSITION_H    =37,
 P_PRESENT_SPEED_L       =38,
 P_PRESENT_SPEED_H       =39,
 P_PRESENT_LOAD_L        =40,
 P_PRESENT_LOAD_H        =41,
 P_PRESENT_VOLTAGE       =42,
 P_PRESENT_TEMPERATURE   =43,
 P_REGISTERED_INSTRUCTION=44,
 P_PAUSE_TIME            =45,
 P_MOVING                =46,
 P_LOCK                  =47,
 P_PUNCH_L               =48,  // Min Torque
 P_PUNCH_H               =49
};

enum ControlTableMX64{
 P_P_GAIN   = 28,
 P_I_GAIN   = 27,
 P_D_GAIN   = 26
};

enum Inst{
 INST_PING       = 1   ,
 INST_READ       = 2   ,
 INST_WRITE      = 3   ,
 INST_REG_WRITE  = 4   ,
 INST_ACTION     = 5   ,
 INST_RESET      = 6   ,
 INST_SYNC_WRITE = 0x83
};


class DynamixelComm : public Serial
{
public:

    DynamixelComm(const char * device_name, unsigned long baud_rate);
    ~DynamixelComm();

    unsigned char   CalculateChecksum(unsigned char * b);
    void            Send(unsigned char * b);
    int             Receive(unsigned char * b);
    bool            ReadAllData(int id, unsigned char * buffer);
    int             Move(int id, int pos, int speed);
    int             SyncState(std::vector<int> id,std::vector<int> pos, std::vector<int> speed);
    int             SetState(int id, int pos, int speed);
    int             SetSpeed(int id, int speed);
    int             SetPosition(int id, int pos);
    int             EnableTorque(int id, int value);
    int             SetLED(int id, int value);
    bool            Ping(int id);
    void            GetPosition(int id, std::vector<int> q);
    void            GetSpeed(int id, std::vector<int> qd);
    void            GetLoad(int id, std::vector<int> u);
    int             SetBaudRate(int id, int br);
    int             ResetBaudRate(int id);
    int             SetReturnLevel(int id, int rl);
    int             SetID(int id1, int id2);
    int             SetTorque(int id, int pos);
    int             SetPositionClamp(int id, int cw, int ccw );
    int             GetPositionClamp(int id, int* cw, int* ccw );

};

#endif

