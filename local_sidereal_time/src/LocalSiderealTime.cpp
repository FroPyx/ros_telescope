#include <local_sidereal_time/LocalSiderealTime.h>
#include <stellar_position_conversion/StellarPositionConversion.h>
#include <iomanip>

/**
** Define the LocalSiderealTime constructor
*/
LocalSiderealTime::LocalSiderealTime()
{

  // Initialize the reference time
  reference_time_greenwich_ = ros::Time(946728000);

  // Initialize the reference sidereal time
  reference_sidereal_time_greenwich_ = 18.697374558;

  // Initialize the local longitude (degree)
  local_longitude_ = 7.261111;

  // Initialize the local_sidereal_time
  local_sidereal_time_ = 0.0;

  // Initialize all local_sidereal_time_publisher
  local_sideral_time_rad_publisher_ = n_.advertise<std_msgs::Float64>("local_sidereal_time/rad", 1, true);
  local_sideral_time_hour_angle_publisher_ = n_.advertise<std_msgs::Float64>("local_sidereal_time/hour_angle", 1, true);
  local_sideral_time_text_publisher_ = n_.advertise<std_msgs::String>("local_sidereal_time/text", 1, true);

}


/**
** Define the LocalSiderealTime initialize function
*/
bool LocalSiderealTime::initialize()
{
  return true;
}


/**
** Define the LocalSiderealTime loopOnce function
*/
void LocalSiderealTime::loopOnce()
{

  // Calculate local sidereal time
  calculateLocalSiderealTime();

  // Publish local sidereal time
  publishLocalSiderealTime();

}


/**
** Define the LocalSiderealTime calculateLocalSiderealTime function
*/
void LocalSiderealTime::calculateLocalSiderealTime()
{
  // Calculate duration between current time and reference time
  ros::Duration time_since_reference_time = ros::Time::now() - reference_time_greenwich_;

  // Convert the duration between current time and reference time in solar time days
  double time_since_reference_time_solar_time_days = static_cast<double>(time_since_reference_time.sec + 1e-9*time_since_reference_time.nsec)/(24.0*60.0*60.0);

  // Calculate current sidereal_time at GMT+0 (each solar time day is 24,06570982441908 sideral days)
  double current_sidereal_time_greenwich = reference_sidereal_time_greenwich_ + time_since_reference_time_solar_time_days*24.06570982441908;

  // Calculate current sidereal_time corrected with longitude
  local_sidereal_time_ = fmod(current_sidereal_time_greenwich + local_longitude_/15.0, 24);
}


/**
** Define the LocalSiderealTime publishLocalSiderealTime function
*/
void LocalSiderealTime::publishLocalSiderealTime()
{
  // Publish local sidereal time in all formats
  publishLocalSiderealTimeHourAngle();
  publishLocalSiderealTimeRad();
  publishLocalSiderealTimeText();
}


/**
** Define the LocalSiderealTime publishLocalSiderealTimeHourAngle function
*/
void LocalSiderealTime::publishLocalSiderealTimeHourAngle()
{
  std_msgs::Float64 local_sidereal_time_hour_angle;
  local_sidereal_time_hour_angle.data = local_sidereal_time_;
  local_sideral_time_hour_angle_publisher_.publish(local_sidereal_time_hour_angle);
}


/**
** Define the LocalSiderealTime publishLocalSiderealTimeRad function
*/
void LocalSiderealTime::publishLocalSiderealTimeRad()
{
  std_msgs::Float64 local_sidereal_time_rad;
  local_sidereal_time_rad.data = hourAngleToRad(local_sidereal_time_);
  local_sideral_time_rad_publisher_.publish(local_sidereal_time_rad);
}


/**
** Define the LocalSiderealTime publishLocalSiderealTimeText function
*/
void LocalSiderealTime::publishLocalSiderealTimeText()
{
  std_msgs::String local_sidereal_time_text;
  local_sidereal_time_text.data = hourAngleToHMS(local_sidereal_time_);
  local_sideral_time_text_publisher_.publish(local_sidereal_time_text);
}
