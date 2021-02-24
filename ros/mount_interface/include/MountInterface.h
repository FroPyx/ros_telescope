#ifndef MOUNT_INTERFACE_
#define MOUNT_INTERFACE_ 1

// Include librairies
#include <ros/ros.h>
#include <serial/serial.h>


// Define the MountInterface class
class MountInterface
{

// Public
public:

  // Define the constructor
  MountInterface();

  // Define the destructor
  ~MountInterface(){}

  // Define the init function
  bool init();

  // Define the loopOnce function
  void loopOnce();


// Private
private:

  // Define the serial port
  std::string serial_port_;

};

#endif
