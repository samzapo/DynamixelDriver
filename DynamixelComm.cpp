//
//    DynamixelComm.cc		Class to communicate with USB2Dynamixel
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
// THIS IS THE NEW COMM

#include <dxl/DynamixelComm.h>

DynamixelComm::DynamixelComm(const char * serial_device, unsigned long baud_rate):
  Serial(serial_device, baud_rate)
{
}


DynamixelComm::~DynamixelComm()
{
}


unsigned char
DynamixelComm::CalculateChecksum(unsigned char * b)
{
  unsigned char checksum = 0;
  for(int i=0; i<(b[3]+1); i++)
    checksum += b[i+2];
  checksum = ~checksum;
  return checksum;
}



void
DynamixelComm::Send(unsigned char * b)
{
  b[b[3]+3] = CalculateChecksum(b);

  SendBytes((char *)b, b[3]+4);
}



int
DynamixelComm::Receive(unsigned char * b)
{
  int c = ReceiveBytes((char *)b, 4);
  fprintf(stdout, "in receive, c = %d b = %s\n", c, b);
  if(c < 4)
  {
    printf("receive error (data size = %d)\n", c);
    return 0;
  }
  c += ReceiveBytes((char *)&b[4], b[3]);

  unsigned char checksum = CalculateChecksum(b);
  fprintf(stdout, "I dunno: %d %d\n", checksum, b[b[3] + 3]);
  if(checksum != b[b[3]+3])
  {
    printf("receive error (data size = %d) incorrect checksum\n", c);
    return -1;
  }

  return c;
}



bool
DynamixelComm::ReadAllData(int id, unsigned char * buffer)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, 0, 50, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);
  int n = Receive(inbuf);

  if(n==0)
    return false;

  // TODO: exit if checksum incorrect

  // TODO: check ID @ inbuf[2]
  // TODO: check ERROR @ inbuf[4], should be 0

  // copy data to buffer

  for(int i=0; i<50; i++)
    buffer[i] = inbuf[i+5];

  return true;
}

int
DynamixelComm::ResetBaudRate(int id){
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_RESET, P_BAUD_RATE, 0x00, 0X00};
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::SetBaudRate(int id, int br){
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_BAUD_RATE, br, 0X00};
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::SetID(int id1, int id2){
  unsigned char outbuf[256] = {0XFF, 0XFF, id1, 4, INST_WRITE, P_ID, id2, 0X00};
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::Move(int id, int pos, int speed)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 7, INST_WRITE, P_GOAL_POSITION_L, pos % 256, pos / 256, speed % 256, speed / 256, 0X00}; // move to position with speed
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::SetState(int id, int pos, int speed)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 7, INST_WRITE, P_GOAL_POSITION_L, pos % 256, pos / 256, speed % 256, speed / 256, 0X00}; // move to position with speed
  unsigned char inbuf[256];
  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::SyncState(int * pos, int * speed)
{
//INST_SYNC_WRITE
  for(int i=0;i<13;i++){
    fprintf(stdout, "%d \t %d\n",pos[i],speed[i]);
    fprintf(stdout, "[0x%X 0x%X] \t [0x%X 0x%X]\n",pos[i] % 0x100, pos[i] / 0x100, speed[i] % 0x100, speed[i] / 0x100);
  }
  unsigned char outbuf[8*((5+1)*13+4 + 8)] =
 //fill, fill, ALL , (L+1)*N+4 ,  command type  , write to (lowest),  L = length,
  {0XFF, 0XFF, 0xFE, (4+1)*13+4, INST_SYNC_WRITE, P_GOAL_POSITION_L, 0x04,
  //ID, pos_L            POS_H            VEL_L              VEL_H
  100, pos[0 ] % 0x100, pos[0 ] / 0x100, speed[0 ] % 0x100, speed[0 ] / 0x100,
    1, pos[1 ] % 0x100, pos[1 ] / 0x100, speed[1 ] % 0x100, speed[1 ] / 0x100,
    2, pos[2 ] % 0x100, pos[2 ] / 0x100, speed[2 ] % 0x100, speed[2 ] / 0x100,
    3, pos[3 ] % 0x100, pos[3 ] / 0x100, speed[3 ] % 0x100, speed[3 ] / 0x100,
    4, pos[4 ] % 0x100, pos[4 ] / 0x100, speed[4 ] % 0x100, speed[4 ] / 0x100,
    5, pos[5 ] % 0x100, pos[5 ] / 0x100, speed[5 ] % 0x100, speed[5 ] / 0x100,
    6, pos[6 ] % 0x100, pos[6 ] / 0x100, speed[6 ] % 0x100, speed[6 ] / 0x100,
    7, pos[7 ] % 0x100, pos[7 ] / 0x100, speed[7 ] % 0x100, speed[7 ] / 0x100,
    8, pos[8 ] % 0x100, pos[8 ] / 0x100, speed[8 ] % 0x100, speed[8 ] / 0x100,
    9, pos[9 ] % 0x100, pos[9 ] / 0x100, speed[9 ] % 0x100, speed[9 ] / 0x100,
   10, pos[10] % 0x100, pos[10] / 0x100, speed[10] % 0x100, speed[10] / 0x100,
   11, pos[11] % 0x100, pos[11] / 0x100, speed[11] % 0x100, speed[11] / 0x100,
   12, pos[12] % 0x100, pos[12] / 0x100, speed[12] % 0x100, speed[12] / 0x100};

  for(int i=0;i<2+(4+1)*13+4;i++)
    fprintf(stdout, " 0x%X ",outbuf[i]);

  unsigned char inbuf[32*8*13];
  Send(outbuf);
//      Receive(inbuf);

  return 1;
}

int
DynamixelComm::SetSpeed(int id, int speed)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_GOAL_SPEED_L, speed % 256, speed / 256, 0X00}; // write two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}



int
DynamixelComm::SetPosition(int id, int pos)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_GOAL_POSITION_L, pos % 256, pos / 256, 0X00}; // write two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);
      Receive(inbuf);

  return 1;
}



int
DynamixelComm::EnableTorque(int id, int value)
{
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_TORQUE_ENABLE, value, 0X00}; // write two bytes for present position
    unsigned char inbuf[256];

    Send(outbuf);

    if(id != 254)
        Receive(inbuf);
  }

  {
    int val = 0x20;//0x200;//0x20; //0x3FF
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_PUNCH_L, val % 256, val / 256, 0x00}; // write two bytes for present position
    Send(outbuf);
  }


  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_CW_COMPLIANCE_MARGIN, 0x04, 0X00}; // write two bytes for present position
    Send(outbuf);
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_CCW_COMPLIANCE_MARGIN, 0x04, 0X00}; // write two bytes for present position
    Send(outbuf);
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_CW_COMPLIANCE_SLOPE, 0x20, 0X00}; // write two bytes for present position
    Send(outbuf);
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_CCW_COMPLIANCE_SLOPE, 0x20, 0X00}; // write two bytes for present position
    Send(outbuf);
  }

  return 1;
}

int
DynamixelComm::SetLED(int id, int value)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_LED, value, 0X00}; // write two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);

  //    if(id != 254)
  //        Receive(inbuf);

  return 1;
}

void DynamixelComm::GetPosition(int id, int q[13])
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_PRESENT_POSITION_L, 2, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];
  inbuf[4] = 0;

  Send(outbuf);

  // set a timout first

  int result = 1;
  while(result > 0 && inbuf[4] == 0){
    result = Receive(inbuf);

    // input result for the recieved packet into its respective address
    q[inbuf[2]] = inbuf[5]+256*inbuf[6];
  }

  // exit if checksum incorrect

  // check ID @ inbuf[2]
  // check ERROR @ inbuf[4], should be 0

}

void DynamixelComm::GetSpeed(int id, int q[13])
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_PRESENT_SPEED_L, 2, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];
  inbuf[4] = 0;

  Send(outbuf);

  // set a timout first

  int result = 1;
  while(result > 0 && inbuf[4] == 0){
    result = Receive(inbuf);

    // input result for the recieved packet into its respective address
    q[inbuf[2]] = inbuf[5]+256*inbuf[6];
  }

  // exit if checksum incorrect

  // check ID @ inbuf[2]
  // check ERROR @ inbuf[4], should be 0

}


void DynamixelComm::GetLoad(int id, int q[13])
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_PRESENT_LOAD_L, 2, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];
  inbuf[4] = 0;

  Send(outbuf);

  // set a timout first

  int result = 1;
  while(result > 0 && inbuf[4] == 0){
    result = Receive(inbuf);

    // input result for the recieved packet into its respective address
    q[inbuf[2]] = inbuf[5]+256*inbuf[6];
  }

  // exit if checksum incorrect

  // check ID @ inbuf[2]
  // check ERROR @ inbuf[4], should be 0

}
/*
int
DynamixelComm::GetSpeed(int id)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_PRESENT_SPEED_L, 2, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);

  // set a timout first

  Receive(inbuf);

  // exit if checksum incorrect

  // check ID @ inbuf[2]
  // check ERROR @ inbuf[4], should be 0

  return inbuf[5]+256*inbuf[6];
}

int
DynamixelComm::GetLoad(int id)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_PRESENT_LOAD_L, 2, 0X00}; // read two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);

  // set a timout first

  Receive(inbuf);

  // exit if checksum incorrect

  // check ID @ inbuf[2]
  // check ERROR @ inbuf[4], should be 0

  return inbuf[5]+256*inbuf[6];
}
*/
int
DynamixelComm::SetTorque(int id, int torque)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_TORQUE_LIMIT_L, torque % 256, torque / 256, 0X00}; // write two bytes for present position
  unsigned char inbuf[256];

  Send(outbuf);
  //    Receive(inbuf);

  return 1;
}

int
DynamixelComm::SetPositionClamp(int id, int cw, int ccw )
{
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_CW_ANGLE_LIMIT_L, cw % 256, cw / 256, 0X00}; // write two bytes for CW position limit
    unsigned char inbuf[256];
    Send(outbuf);
    //        Receive(inbuf);
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 5, INST_WRITE, P_CCW_ANGLE_LIMIT_L, ccw % 256, ccw / 256, 0X00}; // write two bytes for CCW position limit
    unsigned char inbuf[256];
    Send(outbuf);
    //        Receive(inbuf);
  }
  return 1;
}

int
DynamixelComm::GetPositionClamp(int id, int* cw, int* ccw )
{
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_CW_ANGLE_LIMIT_L, 2, 0X00}; // read two bytes for present position
    unsigned char inbuf[256];

    Send(outbuf);

    // set a timout first

    Receive(inbuf);

    // exit if checksum incorrect

    // check ID @ inbuf[2]
    // check ERROR @ inbuf[4], should be 0

    *cw = inbuf[5]+256*inbuf[6];
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_READ, P_CCW_ANGLE_LIMIT_L, 2, 0X00}; // read two bytes for present position
    unsigned char inbuf[256];

    Send(outbuf);

    // set a timout first

    Receive(inbuf);

    // exit if checksum incorrect

    // check ID @ inbuf[2]
    // check ERROR @ inbuf[4], should be 0

    *ccw = inbuf[5]+256*inbuf[6];
  }
  return 0;
}


int  DynamixelComm::SetReturnLevel(int id, int value){
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_RETURN_LEVEL, value, 0X00}; // write two bytes for present position
    unsigned char inbuf[256];

    Send(outbuf);

    //        if(id != 254)
    //            Receive(inbuf);
  }
  {
    unsigned char outbuf[256] = {0XFF, 0XFF, id, 4, INST_WRITE, P_RETURN_DELAY_TIME, 0, 0X00}; // write two bytes for present position
    unsigned char inbuf[256];

    Send(outbuf);

    //        if(id != 254)
    //            Receive(inbuf);
  }
  return 1;
}

// ping a dynamixel to see if it exists

bool
DynamixelComm::Ping(int id)
{
  unsigned char outbuf[256] = {0XFF, 0XFF, id, 2, INST_PING, 0X00};
  unsigned char inbuf[256];

  fprintf(stdout, "About to ping %d with %s\n", id, outbuf);

  Send(outbuf);
  int n = Receive(inbuf);

  return n;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#define USE_SOCKET
#define IKAROSPATH	"../"
#define MAX_FAILED_READS 1

#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif



Serial::Serial(const char * device_name, unsigned long baud_rate)
{
  struct termios options;

  fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if(fd == -1)
    throw SerialException("Could not open serial device.\n", errno);

  fcntl(fd, F_SETFL, 0); // blocking
  tcgetattr(fd, &options); // get the current options // TODO: restore on destruction of the object

#ifndef __APPLE__
  if(cfsetispeed(&options, baud_rate))
    throw SerialException("Could not set baud rate for input", errno);
  if(cfsetospeed(&options, baud_rate))
    throw SerialException("Could not set baud rate for output", errno);
#endif

  options.c_cflag |= (CS8 | CLOCAL | CREAD);
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 1;    // tenth of seconds allowed between bytes of serial data
  // but since VMIN = 0 we will wait at most 1/10 s for data then return
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &options);   // set the options

#ifdef __APPLE__
  cfmakeraw(&options); // necessary for ioctl to function; must come after setattr
  const speed_t TGTBAUD = baud_rate;
  int ret = ioctl(fd, IOSSIOSPEED, &TGTBAUD); // sets also non-standard baud rates
  if (ret)
    throw SerialException("Could not set baud rate", errno);
#endif
}



Serial::~Serial()
{
  Close();
}


void
Serial::Flush()
{
  tcflush(fd, TCIOFLUSH);
}



void
Serial::FlushOut()
{
  tcflush(fd, TCOFLUSH);
}



void
Serial::FlushIn()
{
  tcflush(fd, TCIFLUSH);
}



// Send a string.
int
Serial::SendString(const char *sendbuf)
{
  if(fd == -1)
    return 0;
  return write(fd, sendbuf, strlen(sendbuf));
}



// Read characters until we recive character c
int
Serial::ReceiveUntil(char *rcvbuf, char c)
{
  if(fd == -1)
    return 0;

  char *bufptr = rcvbuf;
  int read_bytes, read_bytes_tot = 0, failed_reads = 0;

  while (failed_reads < MAX_FAILED_READS)
  {
    read_bytes = read(fd, bufptr, 1);

    if (read_bytes == 0)
    {
      failed_reads++;
      continue;
    }

    if (read_bytes > 0)
    {
      bufptr += read_bytes;
      read_bytes_tot += read_bytes;
    }

    if (read_bytes < 0)
      break;

    if (bufptr[-1] == c)
      break;
  }

  if (read_bytes_tot == 0)
    return read_bytes;
  else
    return read_bytes_tot;
}



int
Serial::SendBytes(const char *sendbuf, int length)
{
  if(fd == -1)
    return 0;

  int n = write(fd, sendbuf, length);

  if(n == -1)
    printf("Could not send bytes", errno);

  return n;
}



int Serial::ReceiveBytes(char *rcvbuf, int length)
{
  if(fd == -1) {
    fprintf(stdout, "COMM Port not initialized\n");
    return 0;
  }

  char *bufptr = rcvbuf;
  int read_bytes, read_bytes_tot = 0, failed_reads = 0;

  while (failed_reads < MAX_FAILED_READS)
  {
    fprintf(stdout, "failed reads = %d\n", failed_reads);
    read_bytes = read(fd, bufptr, length - read_bytes_tot);

    if (read_bytes == 0)
    {
      failed_reads++;
      continue;
    }

    if (read_bytes < 0)
      break;

    bufptr += read_bytes;
    read_bytes_tot += read_bytes;

    if (read_bytes_tot >= length)
      break;
  }

  if (read_bytes_tot == 0)
    return read_bytes;
  else
    return read_bytes_tot;
}



void Serial::Close()
{
  Flush();
  if(fd != -1)
    close(fd);
}

#undef MAX_FAILED_READS
#undef USE_SOCKET
#undef IKAROSPATH
