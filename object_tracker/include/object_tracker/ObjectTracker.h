#ifndef _OBJECT_TRACKER_
#define _OBJECT_TRACKER_ 1

// Include libraries
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <telescope_msgs/MotorsPositionCommand.h>
#include <telescope_msgs/MotorsStatus.h>
#include <telescope_msgs/PositionHourAngleTextFormat.h>
#include <telescope_msgs/SpeedCommand.h>
#include <stellar_position_conversion/StellarPositionConversion.h>


enum TrackStatus
{
  NOTHING = 0,
  MANUAL = 1,
  MANUAL_TRACKING = 2,
  DSO = 3,
  SSO = 4,
  LUNAR = 5,
  SOLAR = 6
};


// Define the ObjectTracker class
class ObjectTracker
{

public:

  // Define the ObjectTracker constructor
  ObjectTracker();

  // Define the initialize function
  bool initialize(ros::NodeHandle &controler_pn);

  // Define the loopOnce function
  void loopOnce();


private:

  // Define the node handler
  ros::NodeHandle n_;

  // Define the track status
  TrackStatus track_status_;

  // Define the reference track position of the object (rad)
  std::vector<double> reference_track_position_;

  // Define the telescope position command (rad)
  std::vector<double> telescope_position_command_;

  // Define the current_telescope_position (rad)
  std::vector<double> current_telescope_position_;

  // Define the rad_to_ra_motor_position
  double rad_to_ra_motor_position_;

  // Define the rad_to_dec_motor_position
  double rad_to_dec_motor_position_;

  // Define the local_sidereal_time
  double local_sidereal_time_;

  // Define the telescope_speed_command
  std::vector<double> telescope_speed_command_;

  // Define time_since_last_telescope_speed_command
  ros::Time time_since_last_telescope_speed_command_;

  // Define the local_sidereal_time subscriber
  ros::Subscriber local_sideral_time_subscriber_;

  // Define the track_request subscriber
  ros::Subscriber track_request_subscriber_;

  // Define the telescope_speed_command_subscriber
  ros::Subscriber telescope_speed_command_subscriber_;

  // Define the ra_motor_position_subscriber
  ros::Subscriber ra_motor_position_subscriber_;

  // Define the dec_motor_position_subscriber
  ros::Subscriber dec_motor_position_subscriber_;

  // Define the current_telescope_position publisher
  ros::Publisher current_telescope_position_publisher_;

  // Define the telescope_position_command publisher
  ros::Publisher telescope_position_command_publisher_;

  // Define the reference_track_position publisher
  ros::Publisher reference_track_position_publisher_;

  // Define the ra_motor_position_command publisher
  ros::Publisher ra_motor_position_command_publisher_;

  // Define the dec_motor_position_command publisher
  ros::Publisher dec_motor_position_command_publisher_;

  // Define the track_status publisher
  ros::Publisher track_status_publisher_;


  // Define the manualControl
  void manualControl();

  // Define the track DSO object
  void trackDSOObject();

  // Define the correctPrecession function
  void correctPrecession(double& ra, double& dec);

  // Define the localSiderealTimeCallback function
  void localSiderealTimeCallback(const std_msgs::Float64& local_sidereal_time_data);

  // Define the trackRequestCallback function
  void trackRequestCallback(const std_msgs::Int16& track_request_data);

  // Define the telescopeSpeedCommandCallback function
  void telescopeSpeedCommandCallback(const telescope_msgs::SpeedCommand& speed_command_data);

  // Define the raMotorPositionCallback function
  void raMotorPositionCallback(const std_msgs::Int32& ra_motor_position_data);

  // Define the decMotorPositionCallback function
  void decMotorPositionCallback(const std_msgs::Int32& dec_motor_position_data);

  // Define the publishCurrentTelescopePosition function
  void publishCurrentTelescopePosition();

  // Define the publishTelescopePositionCommand function
  void publishTelescopePositionCommand();

  // Define the publishTelescopePositionCommand function
  void publishReferenceTrackPosition();

  // Define the publishMotorsCommand function
  void publishMotorsCommand();

}; // ObjectTracker


#endif // _OBJECT_TRACKER_
