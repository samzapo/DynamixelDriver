#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H
#include <unistd.h>
#include <iostream>
#include <cassert>
#include <math.h>
#include <vector>

namespace DXL{
const int
    // Body Sagittal Flexion
    // Hip Coronal Flexion
BODY_JOINT = 0,
    LF_X_1 = 1,
    RF_X_1 = 2,
    LH_X_1 = 3,
    RH_X_1 = 4,
    // Hip Sagittal Flexion
    LF_Y_2 = 5,
    RF_Y_2 = 6,
    LH_Y_2 = 7,
    RH_Y_2 = 8,
    // LEG Sagittal Flexion
    LF_Y_3 = 9,
    RF_Y_3 = 10,
    LH_Y_3 = 11,
    RH_Y_3 = 12,
    // Num int
    N_JOINTS = 13;

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
  Dynamixel(){}
  Dynamixel(const char * device_name, unsigned long baud_rate = 1000000);
   void set_state(const double* q,const double* qd);
   void get_state(  double * q, double * qd, double * u);
   void set_position(const double* q);
   void set_velocity(const double* qd);
   void set_torque(const double* t);
   void set_joint_limits(const double* cw_lower,const double* ccw_upper);
   void relaxed(bool torque_off);
  static std::string JointName(int j){
    switch(j){
    case LF_X_1:return "0LF_X_1";
    case RF_X_1:return "0RF_X_1";
    case LH_X_1:return "0LH_X_1";
    case RH_X_1:return "0RH_X_1";
    case LF_Y_2:return "0LF_Y_2";
    case RF_Y_2:return "0RF_Y_2";
    case LH_Y_2:return "0LH_Y_2";
    case RH_Y_2:return "0RH_Y_2";
    case LF_Y_3:return "0LF_Y_3";
    case RF_Y_3:return "0RF_Y_3";
    case LH_Y_3:return "0LH_Y_3";
    case RH_Y_3:return "0RH_Y_3";
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
