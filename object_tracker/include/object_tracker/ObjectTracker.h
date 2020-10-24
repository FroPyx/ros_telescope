#ifndef _OBJECT_TRACKER_
#define _OBJECT_TRACKER_ 1

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

  // Define the rad_to_ra_motor_position
  double rad_to_ra_motor_position_;

  // Define the rad_to_dec_motor_position
  double rad_to_dec_motor_position_;



}; // ObjectTracker


#endif // _OBJECT_TRACKER_
