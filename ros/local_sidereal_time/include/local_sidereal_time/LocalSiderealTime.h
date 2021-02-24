#ifndef _LOCAL_SIDEREAL_TIME_
#define _LOCAL_SIDEREAL_TIME_ 1

// Include libraries
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


// Define the LocalSiderealTime class
class LocalSiderealTime
{

public:

  // Define the LocalSiderealTime constructor
  LocalSiderealTime();

  // Define the initialize function
  bool initialize();

  // Define the loopOnce function
  void loopOnce();


private:

  // Define the node handler
  ros::NodeHandle n_;

  // Define the local_sidereal_time_hour_angle_publisher
  ros::Publisher local_sideral_time_hour_angle_publisher_;

  // Define the local_sidereal_time_rad_publisher
  ros::Publisher local_sideral_time_rad_publisher_;

  // Define the local_sidereal_time_text_publisher
  ros::Publisher local_sideral_time_text_publisher_;

  // Define the refererence timestamp of 01/01/2000 12h GMT+0
  ros::Time reference_time_greenwich_;

  // Define the reference sidereal time of reference time
  double reference_sidereal_time_greenwich_;

  // Define the local longitute
  double local_longitude_;

  // Define the local_sidereal_time
  double local_sidereal_time_;


  // Define the calculateLocalSiderealTime function
  void calculateLocalSiderealTime();

  // Define the publishLocalSiderealTime function
  void publishLocalSiderealTime();

  // Define the publishLocalSiderealTimeHourAngle function
  void publishLocalSiderealTimeHourAngle();

  // Define the publishLocalSiderealTimeRad function
  void publishLocalSiderealTimeRad();

  // Define the publishLocalSiderealTimeText function
  void publishLocalSiderealTimeText();


}; // LocalSiderealTime


#endif // _LOCAL_SIDEREAL_TIME_
