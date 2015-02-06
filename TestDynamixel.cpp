#include <dxl/Dynamixel.h>
#include <Ravelin/VectorNd.h>
using namespace DXL;
#ifdef __APPLE__
# define DEVICE_NAME "/dev/tty.usbserial-A9YL9ZZV"
#elif __arm__
# define DEVICE_NAME "/dev/ttyUSB0"
#endif

int main(int argc,char* argv[]){
  std::cout << "Starting DXL Test " << std::endl;
  
  std::cout << "device: " << DEVICE_NAME << std::endl;
  Dynamixel::init(DEVICE_NAME,57600);
  std::cout << "Device inited" << std::endl;
  

  Ravelin::VectorNd q,qd,u;

  Dynamixel::get_state(q.data(),qd.data(),u.data());

  std::cout << "q: " << q << std::endl;
  std::cout << "qd: " << qd << std::endl;

  return 0;
}
