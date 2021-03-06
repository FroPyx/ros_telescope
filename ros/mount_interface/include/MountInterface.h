#ifndef MOUNT_INTERFACE_
#define MOUNT_INTERFACE_ 1

// Include librairies
#include <ros/ros.h>
#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>


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
  bool init(const ros::NodeHandle& private_node_handler);

  // Define the loopOnce function
  void loopOnce();


// Private
private:


  // Define the node handler
  ros::NodeHandle node_handler_;

  // Define the mount_motor_position_command_subscriber
  ros::Subscriber mount_ra_motor_position_command_subscriber_;

  // Define the mount_dec_position_command_subscriber
  ros::Subscriber mount_dec_motor_position_command_subscriber_;

  // Define the mount_reset_motors_position_command_subscriber
  ros::Subscriber mount_reset_motors_position_command_subscriber_;

  // Define the mount_enable_motors_power_command_subscriber
  ros::Subscriber mount_enable_motors_power_command_subscriber_;

  // Define the mount_ra_motor_position_publisher
  ros::Publisher mount_ra_motor_position_publisher_;

  // Define the mount_dec_motor_position_publisher
  ros::Publisher mount_dec_motor_position_publisher_;

  // Define the mount_enable_motors_power_publisher
  ros::Publisher mount_enable_motors_power_publisher_;


  // Define the serial interface
  serial::Serial serial_interface_;

  // Define the serial port
  std::string serial_port_;

  // Define the serial baudrate
  int serial_baudrate_;

  // Define the serial timeout
  float serial_timeout_;

  // Define the serial_data
  std::vector<std::string> serial_data_;

  // Define the serial_command
  std::vector<std::string> serial_command_;


  // Define the getSerialData function
  void getSerialData();

  // Define the updateSerialData function
  void updateSerialData();

  // Define the sendSerialCommand function
  void sendSerialCommand();

  // Define the mountRaMotorPositionCommandCallback function
  void mountRaMotorPositionCommandCallback(const std_msgs::Int32& ra_motor_position_command_data);

  // Define the mountDecMotorPositionCommandCallback function
  void mountDecMotorPositionCommandCallback(const std_msgs::Int32& dec_motor_position_command_data);

  // Define the mountResetMotorsPositionCommandCallback function
  void mountResetMotorsPositionCommandCallback(const std_msgs::Empty& reset_motors_position_command_data);

  // Define the mountEnableMotorsPowerCommandCallback function
  void mountEnableMotorsPowerCommandCallback(const std_msgs::Bool& enable_motors_power_command_data);

};

#endif
