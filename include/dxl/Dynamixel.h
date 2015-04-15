#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H
#include <unistd.h>
#include <iostream>
#include <cassert>
#include <math.h>
#include <vector>

const double MX_64R_MAXTORQUE = 6.0;
const double RX_24F_MAXTORQUE = 2.6;

const int MX_64R_MAXUNIT = 4094; //4095
const int RX_24F_MAXUNIT = 1023; //1024

// Position values
const double MX_64R_UNIT2RAD = 0.00153588974;
const double RX_24F_UNIT2RAD = 0.00506145483;
const double MX_64R_RAD2UNIT = 651.088404302;
const double RX_24F_RAD2UNIT = 197.571653524;

namespace DXL{
inline double wrapAngle2( double angle )
{
    double twoPi = 2.0 * M_PI;
    return angle - twoPi * floor( angle / twoPi );
}

inline double wrapAngle( double angle )
{
    if(angle < -M_PI || M_PI < angle)
        angle = wrapAngle2(angle + M_PI) - M_PI;
    return angle;
}

inline double sign( double x ){
    return (x>=0)? 1.0 : -1.0;
}

class Dynamixel{
private:
  std::vector<double> workv1d, workv2d;
  std::vector<int> workv1i, workv2i;

public:
  enum Type{
    RX_24F = 0,
    MX_64R = 1
  };
  std::vector<int> ids;
  std::vector<std::string> names;
  std::vector<int> tare;
  std::vector<Type> stype;

  int maxUnit(int id);
  int centerUnit(int id);

  Dynamixel(){}
  Dynamixel(const char * device_name, unsigned long baud_rate = 1000000);
   void set_state(const std::vector<double> q,const std::vector<double> qd);
   void get_state(  std::vector<double> q, std::vector<double> qd, std::vector<double> u);
   void set_position(const std::vector<double> q);
   void set_velocity(const std::vector<double> qd);
   void set_torque(const std::vector<double> t);
   void set_joint_limits(const std::vector<double> cw_lower,const std::vector<double> ccw_upper);
   void relaxed(bool torque_off);
   std::string JointName(int i){
     return names[i];
   }

  /// rad 2 int
  int convert_position(int i,double q);

  /// int 2 rad
  double convert_position(int i,int q);

  /// int 2 rad
  double convert_velocity(int i,int qd);

  /// rad 2 int
  int convert_velocity(int i,double qd);
};
} // end namespace DXL
#endif // DYNAMIXEL_H
