#include <dxl/Dynamixel.h>

#ifdef __APPLE__
# define DEVICE_NAME "/dev/tty.usbserial-A9YL9ZZV"
#elif __arm__
# define DEVICE_NAME "/dev/ttyUSB0"
#endif

int main(int argc,char* argv[]){
  DXL::Dynamixel::init(DEVICE_NAME,1000000);
 
     

  return 0;
}
