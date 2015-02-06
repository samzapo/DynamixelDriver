#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H
#include <unistd.h>
#include <iostream>
#include <cassert>
#include <math.h>
#include <vector>

namespace DXL{
const int
    // Hip Coronal Flexion
    LF_HIP_AA = 1,
    RF_HIP_AA = 2,
    LH_HIP_AA = 3,
    RH_HIP_AA = 4,
    // Hip Sagittal Flexion
    LF_HIP_FE = 5,
    RF_HIP_FE = 6,
    LH_HIP_FE = 7,
    RH_HIP_FE = 8,
    // LEG Sagittal Flexion
    LF_LEG_FE = 9,
    RF_LEG_FE = 10,
    LH_LEG_FE = 11,
    RH_LEG_FE = 12,
    // Num int
    N_JOINTS = 12;

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
public:
  static void set_state(const double* q,const double* qd);
  static void get_state(  double * q, double * qd, double * u);
  static void set_position(const double* q);
  static void set_velocity(const double* qd);
  static void set_torque(const double* t);
  static void set_joint_limits(const double* cw_lower,const double* ccw_upper);
  static void relaxed(bool torque_off);
  static std::string JointName(int j){
    switch(j){
    case LF_HIP_AA:return "LF_HIP_AA";
    case RF_HIP_AA:return "RF_HIP_AA";
    case LH_HIP_AA:return "LH_HIP_AA";
    case RH_HIP_AA:return "RH_HIP_AA";
    case LF_HIP_FE:return "LF_HIP_FE";
    case RF_HIP_FE:return "RF_HIP_FE";
    case LH_HIP_FE:return "LH_HIP_FE";
    case RH_HIP_FE:return "RH_HIP_FE";
    case LF_LEG_FE:return "LF_LEG_FE";
    case RF_LEG_FE:return "RF_LEG_FE";
    case LH_LEG_FE:return "LH_LEG_FE";
    case RH_LEG_FE:return "RH_LEG_FE";
    default: return "Unknown Joint";
    }
  }

  /// rad 2 int
  static int convert_position(int i,double q);

  /// int 2 rad
  static double convert_position(int i,int q);

  /// int 2 rad
  static double convert_velocity(int i,int qd);

  /// rad 2 int
  static int convert_velocity(int i,double qd);

  static void init(const char * device_name, unsigned long baud_rate);
};
} // end namespace DXL
#endif // DYNAMIXEL_H
