#include <dxl/Dynamixel.h>
#include <dxl/DynamixelComm.h>
#include <iomanip>      // std::setprecision

#define MX_64R_MAXTORQUE 6.0
#define RX_24F_MAXTORQUE 2.6

#define MX_64R_MAXUNIT 4094 //4095
#define RX_24F_MAXUNIT 1023 //1024

// Position values
#define MX_64R_UNIT2RAD 0.00153588974
#define RX_24F_UNIT2RAD 0.00506145483
#define MX_64R_RAD2UNIT 651.088404302
#define RX_24F_RAD2UNIT 197.571653524

// Speed Values
// NOTE: moving speed to Goal Position:
// sign(q - q_des);
#define MX_64R_SPEED2RPS 0.01162389280499438 // 1 unit = 0.111rpm = 0.01162389280499438 rad/sec
#define RX_24F_SPEED2RPS 0.01193805206999423 // 1 unit = 0.114rpm = 0.01193805206999423 rad/sec
#define MX_64R_RPS2SPEED 86.0296990669
#define RX_24F_RPS2SPEED 83.7657596178
using namespace DXL;
const double RX_CENTER               = 512;
const double MX_CENTER               = 2047;
const double zeros[N_JOINTS]         = {0,  0,0,0,0,  0,0,0,0,  0,0,0,0};
const int izeros[N_JOINTS]         = {0,  0,0,0,0,  0,0,0,0,  0,0,0,0};
const int    q_tare[N_JOINTS]        = {RX_CENTER,  RX_CENTER,RX_CENTER,RX_CENTER,RX_CENTER,  RX_CENTER,RX_CENTER,MX_CENTER-60,MX_CENTER-190, RX_CENTER,RX_CENTER,RX_CENTER,RX_CENTER};
const int    q_offset[N_JOINTS]      = {0,  0,0,0,0,  RX_CENTER/4,-RX_CENTER/4,-MX_CENTER/4,MX_CENTER/4, RX_CENTER/2 + 100,-RX_CENTER/2 - 100,-RX_CENTER/2 - 100,RX_CENTER/2 + 100};
const double q_max[N_JOINTS]         = {RX_24F_MAXUNIT,  RX_24F_MAXUNIT,RX_24F_MAXUNIT,RX_24F_MAXUNIT,RX_24F_MAXUNIT,   RX_24F_MAXUNIT,RX_24F_MAXUNIT,MX_64R_MAXUNIT,MX_64R_MAXUNIT,   RX_24F_MAXUNIT,RX_24F_MAXUNIT,RX_24F_MAXUNIT,RX_24F_MAXUNIT};
const double q_init[N_JOINTS]        = {M_PI/6,  M_PI/6,M_PI/6,M_PI/6,M_PI/6,  M_PI/6,M_PI/6,0,0,M_PI/6,M_PI/6,M_PI/6,M_PI/6};

const int
    RX_24F = 0,
    MX_64R = 1;

const int stype[N_JOINTS] = {RX_24F,RX_24F,RX_24F,RX_24F,RX_24F,RX_24F,RX_24F,MX_64R,MX_64R,RX_24F,RX_24F,RX_24F,RX_24F};

const int ALL_SERVOS = 0xFE;

static DynamixelComm* dxl_;

/// rad 2 int
 int Dynamixel::convert_position(int i,double q){
  // convert from radians to integer units
  int q_val = q * ((stype[i] == MX_64R)? MX_64R_RAD2UNIT : RX_24F_RAD2UNIT);
  // center at robot 0 position
  q_val += (q_tare[i] - q_offset[i]);
  // clamp to min and max values
  if(q_val > q_max[i]) q = q_max[i];
  else if(q_val < 0) q = 0;
  return q_val;
}

/// int 2 rad
 double Dynamixel::convert_position(int i,int q){
  q -= (q_tare[i] - q_offset[i]);
  // convert from integer units to radians
  return double((q) * ((stype[i] == MX_64R)? MX_64R_UNIT2RAD : RX_24F_UNIT2RAD));
}

/// int 2 rads
 double Dynamixel::convert_velocity(int i,int qd){
  return qd * ((stype[i] == MX_64R)? MX_64R_SPEED2RPS : RX_24F_SPEED2RPS);
}

/// rad 2 int
 int Dynamixel::convert_velocity(int i,double qd){
  return abs(qd) * ((stype[i] == MX_64R)? MX_64R_RPS2SPEED : RX_24F_RPS2SPEED);
}

void Dynamixel::init(const char * device_name, unsigned long baud_rate){
  dxl_ = new DynamixelComm(device_name,baud_rate);
  dxl_->SetReturnLevel(ALL_SERVOS,1);      // Return only for the READ command
  dxl_->EnableTorque(ALL_SERVOS, 1);
  int speed[N_JOINTS] = {100,  100,100,100,100,  100,100,100,100,  100,100,100,100};
  dxl_->SyncState((int*)q_tare,(int*)speed);
  sleep(3);
  std::cout << "Robot was inited from Dynamixels at: " << device_name << std::endl;
}

void Dynamixel::relaxed(bool torque_off){
  dxl_->EnableTorque(ALL_SERVOS,(torque_off)?0:1);
}

/// set velocity (to goal pos)
void Dynamixel::set_velocity(const double qd[N_JOINTS]){
  static int valqd;
  for(int i=BODY_JOINT;i<N_JOINTS;i++){
    valqd = (stype[i] == MX_64R)? qd[i]*MX_64R_RPS2SPEED : /* else RX-24F */ qd[i]*RX_24F_RPS2SPEED;
    dxl_->SetTorque(i,1023);
    dxl_->SetSpeed(i,valqd);
  }
}

/// set goal pose
void Dynamixel::set_position(const double * q){
  int qi[N_JOINTS];
  for(int i=BODY_JOINT;i<N_JOINTS;i++){
    qi[i] = convert_position(i,q[i]);
  }
  dxl_->SyncState((int*)qi,(int*)izeros);
}

void Dynamixel::set_state(const double * q,const double * qd){
  int qi[N_JOINTS], qid[N_JOINTS];
  for(int i=BODY_JOINT;i<N_JOINTS;i++){
    qi[i] = convert_position(i,q[i]);
    qid[i] = convert_velocity(i,qd[i]);
  }

  dxl_->SyncState(qi,qid);
}

void Dynamixel::set_joint_limits(const double* cw_lower,const double* ccw_upper){
  for(int i=BODY_JOINT;i<N_JOINTS;i++)
    dxl_->SetPositionClamp(i,convert_position(i,cw_lower[i]),convert_position(i,ccw_upper[i]));
}

void Dynamixel::get_state( double * q, double * qd, double * u){
  int qi[N_JOINTS], qid[N_JOINTS], ui[N_JOINTS];
  dxl_->GetPosition(ALL_SERVOS,qi);
  dxl_->GetSpeed(ALL_SERVOS,qid);
  dxl_->GetLoad(ALL_SERVOS,ui);
  for(int i=BODY_JOINT;i<N_JOINTS;i++){
    q[i] = convert_position(i,qi[i]);
    qd[i] = convert_velocity(i,qid[i]);
    u[i] = ui[i];
  }
}

// note, this calls N_JOINTS services (very slow)
/*
void Dynamixel::get_robot_state(){
    unsigned char * buffer[N_JOINTS];
    for(int i=BODY_JOINT;i<N_JOINTS;i++){
        ReadAllData(i,buffer[i]);
        for(int i=ID;i<N_OPTIONS;i++)
            std::string translateValue(buffer[i]);
    }
}

std::string translateValue(unsigned int value atAddress, unsigned int address)
{
    switch(address)
    {
        case 0: // Model Number
            switch(value)
            {
                case 24:   return @"RX-24F";
                case 64:   return @"MX-64R";
                default:   return @"Unkown dynamixel";
            }

            return (value == 12 ? @"AX-12" : @"Unkown dynamixel");

        case 4: // Buad Rate
            if(value == 1)
                return @"1 Mbps";
            else if(value <10)
                return [[NSString alloc] initWithFormat:@"%d kbps", 2000/(1+value)];
            else
                return [[NSString alloc] initWithFormat:@"%d bps", 2000000/(1+value)];

        case 5: // Return Delay Time
            return [[NSString alloc] initWithFormat:@"%d μs", 2*value];

        case 16: // Status Return Level
            switch(value)
            {
                case 0:  return @"Never";
                case 1:  return @"READ_DATA only";
                case 2:  return @"Always";
                default: return @"Illegal value";
            }

        case 24: // ON/OFF
        case 25:
            return (value == 0 ? @"Off" : @"On");

        case 17: // Status Return Level
        case 18: // Alarm Shutdown
            return [[NSString alloc] initWithFormat:@"%d", value]; // Check the bits here => IOCRHAV

        case 11: // Temperatures
        case 43:
            return [[NSString alloc] initWithFormat:@"%d °C", value];

        case 12: // Voltages
        case 13:
        case 42:
            return [[NSString alloc] initWithFormat:@"%.1f V", float(value)/10];

        case 38: // Speed/Load
        case 40:
            if(value == 0)
                return @"0";
            else if(value < 1024)
                return [[NSString alloc] initWithFormat:@"+%d", value];
            else
                return [[NSString alloc] initWithFormat:@"-%d", value-1024];

        case 6: // Angles
        case 8:
        case 30:
        case 36:
        case 26: // We assume that the correct unit for the compliance parameters is in degrees
        case 27:
        case 28:
        case 29:
            return [[NSString alloc] initWithFormat:@"%.1f°", 300.0f*float(value)/float(0x3ff)];

        case 32: //Speed
            if(value > 0)
                return [[NSString alloc] initWithFormat:@"%.1f RPM", 114.0f*float(value)/float(0x3ff)];
            else
                return @"Max RPM";

        case 48: // Punch
            return [[NSString alloc] initWithFormat:@"%.1f%%", 100.0f*float(value)/float(0x3ff)];

        default:
            return [[NSString alloc] initWithFormat:@"%d", value];
    }
}
*/



void Dynamixel::set_torque(const double* t){
  static int valt,valq;
  for(int i=BODY_JOINT;i<N_JOINTS;i++){
    valt = (stype[i] == MX_64R)? (fabs(t[i])/MX_64R_MAXTORQUE)*1023 : (fabs(t[i])/RX_24F_MAXTORQUE)*1023;
    if(valt > 1023) valt = 1023;
    valq = (t[i]>=0)? ((stype[i] == MX_64R)? MX_64R_MAXUNIT:RX_24F_MAXUNIT) : 0;
    dxl_->SetState(i,valq,0);
    dxl_->SetTorque(i,valt);
  }
}

int main(int argc,char* argv[]){
  bool use_class = true;
  if(use_class){
    Dynamixel::init("/dev/tty.usbserial-A9YL9ZZV",1000000);
  } else {
    int speed[N_JOINTS] = {100,  100,100,100,100,  100,100,100,100,  100,100,100,100};
    dxl_ = new DynamixelComm("/dev/tty.usbserial-A9YL9ZZV",1000000);
    dxl_->SetReturnLevel(ALL_SERVOS,1);      // Return only for the READ command
    dxl_->EnableTorque(ALL_SERVOS, 1);

    dxl_->SyncState((int*)q_tare,(int*)speed);

    for(int i=0;i<N_JOINTS;i++){
      fprintf(stdout, "Pinging: %d\n", dxl_->Ping(i));
//      dxl_->SetState(i, q_tare[i],0);
    }
  }

  return 0;
}

#undef MX_64R_MAXTORQUE
#undef RX_24F_MAXTORQUE

#undef MX_64R_MAXUNIT
#undef RX_24F_MAXUNIT

#undef MX_64R_UNIT2RAD
#undef RX_24F_UNIT2RAD
#undef MX_64R_RAD2UNIT
#undef RX_24F_RAD2UNIT

#undef MX_64R_SPEED2RPS
#undef RX_24F_SPEED2RPS
#undef MX_64R_RPS2SPEED
#undef RX_24F_RPS2SPEED

#undef ALL_SERVOS
