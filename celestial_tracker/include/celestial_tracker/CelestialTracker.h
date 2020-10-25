#ifndef CELESTIAL_TRACKER_H
#define CELESTIAL_TRACKER_H 1

// Include libraries
#include <ros/ros.h>
#include <stellar_position_conversion/StellarPositionConversion.h>
#include <telescope_msgs/ChangeTrackingMode.h>


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

  // Define the change_tracking_mode_service_server
  ros::ServiceServer change_tracking_mode_service_server_;


  // Define the telescope_position
  telescope_msgs::EquatorialCoordinates telescope_position_;

  // Define the telescope_position_command
  telescope_msgs::EquatorialCoordinates telescope_position_command_;

  // Define the rad_to_ra_motor_position
  double rad_to_ra_motor_position_;

  // Define the rad_to_ra_motor_position
  double rad_to_dec_motor_position_;

  // Define the current_tracking_mode
  uint8_t current_tracking_mode_;


  // Define the telescopePositionCommandCallback function
  void telescopePositionCommandCallback(const telescope_msgs::EquatorialCoordinates& telescope_position_command_data);

  // Define the changeTrackingModeServiceCallback function
  bool changeTrackingModeServiceCallback(telescope_msgs::ChangeTrackingModeRequest& req, telescope_msgs::ChangeTrackingModeResponse& res);


}; // CelestialTracker


#endif // CELESTIAL_TRACKER_H
