#include <dxl/Dynamixel.h>
#include <dxl/DynamixelComm.h>
#include <iomanip>      // std::setprecision

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

const int ALL_SERVOS = 0xFE;
static DynamixelComm* dxl_;

int Dynamixel::maxUnit(int id){
  return ((stype[id] == MX_64R)? MX_64R_MAXUNIT : RX_24F_MAXUNIT);
}

int Dynamixel::centerUnit(int id){
 return ((stype[id] == MX_64R)? MX_CENTER : RX_CENTER);
}


/// rad 2 int
 int Dynamixel::convert_position(int i,double q){
  // convert from radians to integer units
  int q_val = q * ((stype[i] == MX_64R)? MX_64R_RAD2UNIT : RX_24F_RAD2UNIT);
  // center at robot 0 position
  q_val += (centerUnit(i) - tare[i]);
  // clamp to min and max values
  if(q_val > maxUnit(i)) q = maxUnit(i);
  else if(q_val < 0) q = 0;
  return q_val;
}

/// int 2 rad
 double Dynamixel::convert_position(int i,int q){
  q -= (centerUnit(i) - tare[i]);
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

Dynamixel::Dynamixel(const char * device_name, unsigned long baud_rate){
  dxl_ = new DynamixelComm(device_name,baud_rate);
//  dxl_->SetReturnLevel(ALL_SERVOS,1);      // Return only for the READ command
//  dxl_->EnableTorque(ALL_SERVOS, 1);
  std::cout << "Robot was inited from Dynamixels at: " << device_name << std::endl;
}

void Dynamixel::relaxed(bool torque_off){
  dxl_->EnableTorque(ALL_SERVOS,(torque_off)?0:1);
}

/// set velocity (to goal pos)
void Dynamixel::set_velocity(const std::vector<double> qd){
  static int valqd;
  for(int i=0;i<ids.size();i++){
    valqd = (stype[i] == MX_64R)? qd[i]*MX_64R_RPS2SPEED : /* else RX-24F */ qd[i]*RX_24F_RPS2SPEED;
    dxl_->SetTorque(i,1023);
    dxl_->SetSpeed(i,valqd);
  }
}

/// set goal pose
void Dynamixel::set_position(const std::vector<double> q){
  std::vector<int> qi(ids.size()), qid(ids.size());;
  for(int i=0;i<ids.size();i++){
    qi[i] = convert_position(i,q[i]);
    qid[i] = 0;
  }
  dxl_->SyncState(ids,qi,qid);
}

void Dynamixel::set_state(const std::vector<double> q,const std::vector<double> qd){
  std::vector<int> qi(ids.size()), qid(ids.size());
  for(int i=0;i<ids.size();i++){
    qi[i] = convert_position(i,q[i]);
    qid[i] = convert_velocity(i,qd[i]);
  }

  dxl_->SyncState(ids,qi,qid);
}

void Dynamixel::set_joint_limits(const std::vector<double> cw_lower,const std::vector<double> ccw_upper){
  for(int i=0;i<ids.size();i++)
    dxl_->SetPositionClamp(i,convert_position(i,cw_lower[i]),convert_position(i,ccw_upper[i]));
}

void Dynamixel::get_state( std::vector<double> q, std::vector<double> qd, std::vector<double> u){
  std::vector<int> qi(ids.size()), qid(ids.size()), ui(ids.size());
  dxl_->GetPosition(ALL_SERVOS,qi);
  dxl_->GetSpeed(ALL_SERVOS,qid);
  dxl_->GetLoad(ALL_SERVOS,ui);
  for(int i=0;i<ids.size();i++){
    q[i] = convert_position(i,qi[i]);
    qd[i] = convert_velocity(i,qid[i]);
    u[i] = ui[i];
  }
}

// note, this calls ids.size() services (very slow)
/*
void Dynamixel::get_robot_state(){
    unsigned char * buffer[ids.size()];
    for(int i=0;i<ids.size();i++){
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



void Dynamixel::set_torque(const std::vector<double> t){
  static int valt,valq;
  for(int i=0;i<ids.size();i++){
    valt = (stype[i] == MX_64R)? (fabs(t[i])/MX_64R_MAXTORQUE)*1023 : (fabs(t[i])/RX_24F_MAXTORQUE)*1023;
    if(valt > 1023) valt = 1023;
    valq = (t[i]>=0)? ((stype[i] == MX_64R)? MX_64R_MAXUNIT:RX_24F_MAXUNIT) : 0;
    dxl_->SetState(i,valq,0);
    dxl_->SetTorque(i,valt);
  }
}

# define DEVICE_NAME "/dev/tty.usbserial-A9YL9ZZV"

#include <Ravelin/VectorNd.h>
int main(int argc,char* argv[]){
  Dynamixel dxl(DEVICE_NAME);

  // LINKS robot
  dxl.tare.push_back(0);
  dxl.tare.push_back(0);
  dxl.tare.push_back(0);
  dxl.tare.push_back(0);

  dxl.tare.push_back(M_PI/4 * RX_24F_RAD2UNIT);
  dxl.tare.push_back(-M_PI/4 * RX_24F_RAD2UNIT);
  dxl.tare.push_back(-M_PI/4 * MX_64R_RAD2UNIT+40);
  dxl.tare.push_back(M_PI/4 * MX_64R_RAD2UNIT+250);

  dxl.tare.push_back(M_PI/2 * RX_24F_RAD2UNIT);
  dxl.tare.push_back(-M_PI/2 * RX_24F_RAD2UNIT);
  dxl.tare.push_back(-M_PI/2 * RX_24F_RAD2UNIT);
  dxl.tare.push_back(M_PI/2 * RX_24F_RAD2UNIT);

  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);

  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::MX_64R);
  dxl.stype.push_back(Dynamixel::MX_64R);

  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);
  dxl.stype.push_back(Dynamixel::RX_24F);


  dxl.names.push_back("0LF_X_1");
  dxl.names.push_back("0RF_X_1");
  dxl.names.push_back("0LH_X_1");
  dxl.names.push_back("0RH_X_1");

  dxl.names.push_back("0LF_Y_2");
  dxl.names.push_back("0RF_Y_2");
  dxl.names.push_back("0LH_Y_2");
  dxl.names.push_back("0RH_Y_2");

  dxl.names.push_back("0LF_Y_3");
  dxl.names.push_back("0RF_Y_3");
  dxl.names.push_back("0LH_Y_3");
  dxl.names.push_back("0RH_Y_3");

  for(int i=1;i<=dxl.names.size();i++){
    dxl.ids.push_back(i);
  }

  double t = 0;
  while(1){
    t += 0.001;
//    std::cout  << sin(t) << std::endl;

    Ravelin::VectorNd zero(dxl.ids.size());
    zero.set_zero();
    for(int i=0;i<dxl.ids.size();i++)
      dxl.set_position(std::vector<double>(zero.begin(),zero.end()));
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
