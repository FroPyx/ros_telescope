#include <object_tracker/ObjectTracker.h>

/**
** Define the ObjectTracker constructor
*/
ObjectTracker::ObjectTracker()
{

  // Initialize track_status_
  track_status_ = TrackStatus::NOTHING;

  // Initialize reference track position
  reference_track_position_.clear();
  reference_track_position_.push_back(0.0);
  reference_track_position_.push_back(0.0);

  // Initialize reference current track position
  telescope_position_command_.clear();
  telescope_position_command_.push_back(0.0);
  telescope_position_command_.push_back(0.0);

  // Initialize current_telescope_position (rad)
  current_telescope_position_.clear();
  current_telescope_position_.push_back(0.0);
  current_telescope_position_.push_back(0.0);

  // Initialize the telescope_speed_command
  telescope_speed_command_.clear();
  telescope_speed_command_.push_back(0.0);
  telescope_speed_command_.push_back(0.0);

  // Initialize the local sidereal time subscriber
  local_sideral_time_subscriber_ = n_.subscribe("local_sidereal_time/rad", 1, &ObjectTracker::localSiderealTimeCallback, this);

  // Initialize the track request subscriber
  track_request_subscriber_ = n_.subscribe("track_request", 1, &ObjectTracker::trackRequestCallback, this);

  // Initialize the telescope speed command subscriber
  telescope_speed_command_subscriber_ = n_.subscribe("telescope_speed_command", 1, &ObjectTracker::telescopeSpeedCommandCallback, this);

  // Initialize the ra_motor_position_subscriber
  ra_motor_position_subscriber_ = n_.subscribe("ra_motor_position", 1, &ObjectTracker::raMotorPositionCallback, this);

  // Initialize the dec_motor_position_subscriber
  dec_motor_position_subscriber_ = n_.subscribe("dec_motor_position", 1, &ObjectTracker::decMotorPositionCallback, this);

  // Initialize the current_telescope_position publisher
  current_telescope_position_publisher_ = n_.advertise<telescope_msgs::PositionHourAngleTextFormat>("current_telescope_position/text", 1, true);

  // Initialize the telescope_position_command publisher
  telescope_position_command_publisher_ = n_.advertise<telescope_msgs::PositionHourAngleTextFormat>("telescope_position_command/text", 1, true);

  // Initialize the telescope_position_command publisher
  reference_track_position_publisher_ = n_.advertise<telescope_msgs::PositionHourAngleTextFormat>("reference_track_position/text", 1, true);

  // Initialize the ra_motor_position_command publisher
  ra_motor_position_command_publisher_ = n_.advertise<std_msgs::Int32>("ra_motor_position_command", 1, true);

  // Initialize the dec_motor_position_command publisher
  dec_motor_position_command_publisher_ = n_.advertise<std_msgs::Int32>("dec_motor_position_command", 1, true);

  // Initialize the track status publisher
  track_status_publisher_ = n_.advertise<std_msgs::Int16>("track_status", 1, true);

  // Initialize rad to ra motor position
  rad_to_ra_motor_position_ = 0.0;

  // Initialize the rad to dec motor position
  rad_to_dec_motor_position_ = 0.0;

}


/**
** Define the ObjectTracker initialize function
*/
bool ObjectTracker::initialize(ros::NodeHandle &controler_pn)
{

  // Get ra motor step per revolution param
  int ra_motor_step_per_revolution;
  if (!controler_pn.getParam("ra_motor_step_per_revolution", ra_motor_step_per_revolution))
  {
    return false;
  }

  // Get dec motor step per revolution param
  int dec_motor_step_per_revolution;
  if (!controler_pn.getParam("dec_motor_step_per_revolution", dec_motor_step_per_revolution))
  {
    return false;
  }

  // Get ra motor microstep param
  int ra_motor_microstep;
  if (!controler_pn.getParam("ra_motor_microstep", ra_motor_microstep))
  {
    return false;
  }

  // Get dec motor microstep param
  int dec_motor_microstep;
  if (!controler_pn.getParam("dec_motor_microstep", dec_motor_microstep))
  {
    return false;
  }

  // Get ra gear ratio
  double ra_gear_ratio;
  if (!controler_pn.getParam("ra_gear_ratio", ra_gear_ratio))
  {
    return false;
  }

  // Get dec gear ratio
  double dec_gear_ratio;
  if (!controler_pn.getParam("dec_gear_ratio", dec_gear_ratio))
  {
    return false;
  }

  // Get ra worm gear ratio
  int ra_worm_gear_ratio;
  if (!controler_pn.getParam("ra_worm_gear_ratio", ra_worm_gear_ratio))
  {
    return false;
  }

  // Get dec worm gear ratio
  int dec_worm_gear_ratio;
  if (!controler_pn.getParam("dec_worm_gear_ratio", dec_worm_gear_ratio))
  {
    return false;
  }

  // Calculate rad_to_ra_motor_position
  rad_to_ra_motor_position_ = ra_motor_step_per_revolution*ra_motor_microstep*ra_gear_ratio*ra_worm_gear_ratio/(2*M_PI);

  // Calculate rad_to_dec_motor_position
  rad_to_dec_motor_position_ = dec_motor_step_per_revolution*dec_motor_microstep*dec_gear_ratio*dec_worm_gear_ratio/(2*M_PI);

  return true;
}


/**
** Define the ObjectTracker loopOnce function
*/
void ObjectTracker::loopOnce()
{
  // Witch track status
  switch(track_status_)
  {
    // No tracking
    case TrackStatus::NOTHING:
      break;

    // Manual tracking
    case TrackStatus::MANUAL:
      manualControl();
      break;

    // Manual tracking (DSO)
    case TrackStatus::MANUAL_TRACKING:
      break;

    // Deep sky object
    case TrackStatus::DSO:
      trackDSOObject();
      break;

    // Solar system object
    case TrackStatus::SSO:
      break;

    // Moon
    case TrackStatus::LUNAR:
      break;

    // Sun
    case TrackStatus::SOLAR:
      break;

  }
  // Publish positions
  publishCurrentTelescopePosition();
  publishReferenceTrackPosition();
  publishTelescopePositionCommand();
  std_msgs::Int16 track_status_message;
  track_status_message.data = track_status_;
  track_status_publisher_.publish(track_status_message);
}


/**
** Define the ObjectTracker manualControl function
*/
void ObjectTracker::manualControl()
{

  // Check if command != 0
  if (telescope_speed_command_[0] != 0.0 || telescope_speed_command_[1] != 0.0)
  {
    // Update telescope_position_command
    telescope_position_command_[0] += telescope_speed_command_[0]/20.0;
    telescope_position_command_[1] += telescope_speed_command_[1]/20.0;

    // Publish motors command
    publishMotorsCommand();
  }

}


/**
** Define the ObjectTracker trackDSOObject function
*/
void ObjectTracker::trackDSOObject()
{

}



/**
** Define the ObjectTracker correctPrecession function
*/
void ObjectTracker::correctPrecession(double &ra, double &dec)
{

  // Initialize duration since J2000
  ros::Duration duration_since_j2000 = ros::Time::now() - ros::Time(946684800);

  // Calculate duration since J2000 in julian century
  double duration_since_j2000_jc = duration_since_j2000.toSec()/3155760000.0;


  // Calculate precession correction parameters (")
  double m = 3.07496 + 0.00186*duration_since_j2000_jc;
  double n = 1.33621 - 0.00057*duration_since_j2000_jc;

  // Tranform correction un rad
  double m_rad = m*M_PI/(180.0*3600.0);
  double n_rad = n*M_PI/(180.0*3600.0);

  // Calculate the delta ra and dec
  double delta_ra = m_rad + n_rad*sin(ra)*tan(dec);
  double delta_dec = n_rad*cos(ra);

  // Correct ra and dec
  ra += delta_ra;
  dec += delta_dec;
  //double ra_hour = current_track_ra_coordinates_*24/(2*M_PI);

}


/**
** Define the ObjectTracker localSiderealTimeCallback function
*/
void ObjectTracker::localSiderealTimeCallback(const std_msgs::Float64& local_sideral_time_data)
{
  // Update local sidereal time data
  local_sidereal_time_ = local_sideral_time_data.data;
}


/**
** Define the ObjectTracker trackRequestCallback function
*/
void ObjectTracker::trackRequestCallback(const std_msgs::Int16& track_request_data)
{
  // Update track status
  switch (track_request_data.data)
  {
    case TrackStatus::NOTHING:
      track_status_ = TrackStatus::NOTHING;
      break;

    case TrackStatus::MANUAL:
      track_status_ = TrackStatus::MANUAL;
      telescope_position_command_ = current_telescope_position_;
      break;

    case TrackStatus::MANUAL_TRACKING:
      track_status_ = TrackStatus::MANUAL_TRACKING;
      break;

    case TrackStatus::DSO:
      track_status_ = TrackStatus::DSO;
      break;

    case TrackStatus::SSO:
      track_status_ = TrackStatus::SSO;
      break;

    case TrackStatus::LUNAR:
      track_status_ = TrackStatus::LUNAR;
      break;

    case TrackStatus::SOLAR:
      track_status_ = TrackStatus::SOLAR;
      break;
  }
}

//1.8/16*4500/3/65

/**
** Define the ObjectTracker telescopeSpeedCommandCallback function
*/
void ObjectTracker::telescopeSpeedCommandCallback(const telescope_msgs::SpeedCommand& speed_command_data)
{
  // Update telescope_speed_command
  telescope_speed_command_ = speed_command_data.data;

  // Update time since last telescope_speed_command
  time_since_last_telescope_speed_command_ = ros::Time::now();
}


/**
** Define the ObjectTracker raMotorPositionCallback function
*/
void ObjectTracker::raMotorPositionCallback(const std_msgs::Int32& ra_motor_position_data)
{
  // Convert RA motor position to telescope RA position
  current_telescope_position_[0] = ra_motor_position_data.data/rad_to_ra_motor_position_;
}


/**
** Define the ObjectTracker decMotorPositionCallback function
*/
void ObjectTracker::decMotorPositionCallback(const std_msgs::Int32& dec_motor_position_data)
{
  // Convert DEC motor position to telescope DEC position
  current_telescope_position_[1] = dec_motor_position_data.data/rad_to_dec_motor_position_;
}


/**
** Define the ObjectTracker publishCurrentTelescopePosition function
*/
void ObjectTracker::publishCurrentTelescopePosition()
{
  // Convert the telescope position (rad) to hour angle text format
  std::vector<std::string> current_telescope_position_hour_angle_text_format = radPositionFormatToHourAngleTextPositionFormat(current_telescope_position_);

  // Initialize the current_telescope_position message
  telescope_msgs::PositionHourAngleTextFormat current_telescope_position_hour_angle_text_format_message;
  current_telescope_position_hour_angle_text_format_message.data = current_telescope_position_hour_angle_text_format;

  // Publish the message
  current_telescope_position_publisher_.publish(current_telescope_position_hour_angle_text_format_message);
}


/**
** Define the ObjectTracker publishTelescopePositionCommand function
*/
void ObjectTracker::publishTelescopePositionCommand()
{
  // Convert the telescope position (rad) to hour angle text format
  std::vector<std::string> telescope_position_command_hour_angle_text_format = radPositionFormatToHourAngleTextPositionFormat(telescope_position_command_);

  // Initialize the current_telescope_position message
  telescope_msgs::PositionHourAngleTextFormat telescope_position_command_hour_angle_text_format_message;
  telescope_position_command_hour_angle_text_format_message.data = telescope_position_command_hour_angle_text_format;

  // Publish the message
  telescope_position_command_publisher_.publish(telescope_position_command_hour_angle_text_format_message);
}


/**
** Define the ObjectTracker publishReferenceTrackPosition function
*/
void ObjectTracker::publishReferenceTrackPosition()
{
  // Convert the telescope position (rad) to hour angle text format
  std::vector<std::string> reference_track_position_hour_angle_text_format = radPositionFormatToHourAngleTextPositionFormat(reference_track_position_);

  // Initialize the current_telescope_position message
  telescope_msgs::PositionHourAngleTextFormat reference_track_position_hour_angle_text_format_message;
  reference_track_position_hour_angle_text_format_message.data = reference_track_position_hour_angle_text_format;

  // Publish the message
  reference_track_position_publisher_.publish(reference_track_position_hour_angle_text_format_message);
}


/**
** Define the ObjectTracker publishMotorsCommand function
*/
void ObjectTracker::publishMotorsCommand()
{
  // Define the motors_position command
  std_msgs::Int32 ra_motor_position_command;
  std_msgs::Int32 dec_motor_position_command;

  // Convert telescope position command to motors position
  ra_motor_position_command.data = static_cast<int32_t>(telescope_position_command_[0]*rad_to_ra_motor_position_);
  dec_motor_position_command.data = static_cast<int32_t>(telescope_position_command_[1]*rad_to_dec_motor_position_);

  // Publish motors command
  ra_motor_position_command_publisher_.publish(ra_motor_position_command);
  dec_motor_position_command_publisher_.publish(dec_motor_position_command);
}
