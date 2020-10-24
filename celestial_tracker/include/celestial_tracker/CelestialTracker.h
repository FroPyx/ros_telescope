#ifndef CELESTIAL_TRACKER_H
#define CELESTIAL_TRACKER_H 1

// Include libraries
#include <ros/ros.h>
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


// Define the CelestialTracker class
class CelestialTracker
{

public:

  // Define the CelestialTracker constructor
  CelestialTracker();

  // Define the initialize function
  bool initialize(ros::NodeHandle &controler_pn);

  // Define the loopOnce function
  void loopOnce();



private:

  // Define the node handler
  ros::NodeHandle n_;

  // Define the telescope_position_text_publisher
  ros::Publisher telescope_position_text_publisher_;

  // Define the telescope_position_rad_publisher
  ros::Publisher telescope_position_rad_publisher_;

  // Define the telescope_position_command_subscriber
  ros::Subscriber telescope_position_command_subscriber_;

  // Define the telescope_position
  telescope_msgs::EquatorialCoordinates telescope_position_;

  // Define the telescope_position_command
  telescope_msgs::EquatorialCoordinates telescope_position_command_;

  // Define the rad_to_ra_motor_position
  double rad_to_ra_motor_position_;

  // Define the rad_to_ra_motor_position
  double rad_to_dec_motor_position_;


  // Define the telescopePositionCommandCallback function
  void telescopePositionCommandCallback(const telescope_msgs::EquatorialCoordinates& telescope_position_command_data);


}; // CelestialTracker


#endif // CELESTIAL_TRACKER_H
