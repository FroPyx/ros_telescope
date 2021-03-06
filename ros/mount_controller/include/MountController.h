#ifndef MOUNT_CONTROLLER_
#define MOUNT_CONTROLLER_ 1

// Include librairies
#include <ros/ros.h>
#include <stellar_position_conversion/StellarPositionConversion.h>
#include <mount_msgs/CelestialCoordinates.h>
#include <mount_msgs/CelestialCoordinatesText.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>


// Define the MountController class
class MountController
{

// Public
public:

  // Define the constructor
  MountController();

  // Define the destructor
  ~MountController(){}

  // Define the init function
  bool init(const ros::NodeHandle& private_node_handler);

  // Define the loopOnce function
  void loopOnce();


// Private
private:

  // Define the node handler
  ros::NodeHandle node_handler_;

  // Define the mount_ra_position_command_subscriber
  ros::Subscriber mount_ra_position_command_subscriber_;

  // Define the mount_dec_position_command_subscriber
  ros::Subscriber mount_dec_position_command_subscriber_;

  // Define the mount_ra_motor_position_subscriber
  ros::Subscriber mount_ra_motor_position_subscriber_;

  // Define the mount_dec_motor_position_subscriber
  ros::Subscriber mount_dec_motor_position_subscriber_;

  // Define the mount_position_rad_publisher
  ros::Publisher mount_position_rad_publisher_;

  // Define the mount_position_ha_publisher
  ros::Publisher mount_position_ha_publisher_;

  // Define the mount_position_text_publisher
  ros::Publisher mount_position_text_publisher_;

  // Define the mount_position_command_rad_publisher
  ros::Publisher mount_position_command_rad_publisher_;

  // Define the mount_position_command_ha_publisher
  ros::Publisher mount_position_command_ha_publisher_;

  // Define the mount_position_command_text_publisher
  ros::Publisher mount_position_command_text_publisher_;

  // Define the mount_ra_motor_position_command_publisher
  ros::Publisher mount_ra_motor_position_command_publisher_;

  // Define the mount_dec_motor_position_command_publisher
  ros::Publisher mount_dec_motor_position_command_publisher_;


  // Define the mount_position_rad
  mount_msgs::EquatorialCoordinates mount_position_rad_;

  // Define the mount_position_rad
  mount_msgs::EquatorialCoordinates mount_position_ha_;

  // Define the mount_position_rad
  mount_msgs::EquatorialCoordinatesText mount_position_text_;

  // Define the mount_position_command_rad
  mount_msgs::EquatorialCoordinates mount_position_command_rad_;

  // Define the mount_position_command_rad
  mount_msgs::EquatorialCoordinates mount_position_command_ha_;

  // Define the mount_position_command_rad
  mount_msgs::EquatorialCoordinatesText mount_position_command_text_;


  // Define the rad_to_mount_ra_motor_ratio
  double rad_to_mount_ra_motor_ratio_;

  // Define the rad_to_mount_dec_motor_ratio
  double rad_to_mount_dec_motor_ratio_;


  // Define the publishMountPosition function
  void publishMountPosition();

  // Define the publishMountCommandPosition function
  void publishMountCommandPosition();

  // Define the publishMountMotorsCommandPosition function
  void publishMountMotorsCommandPosition();

  // Define the mountRaPositionCommandCallback function
  void mountRaPositionCommandCallback(const std_msgs::Float64& mount_ra_position_command_data);

  // Define the mountDecPositionCommandCallback function
  void mountDecPositionCommandCallback(const std_msgs::Float64& mount_dec_position_command_data);

  // Define the mountRaMotorPositionCallback function
  void mountRaMotorPositionCallback(const std_msgs::Int32& mount_ra_motor_position_data);

  // Define the mountDecMotorPositionCallback function
  void mountDecMotorPositionCallback(const std_msgs::Int32& mount_dec_motor_position_data);

};

#endif
