#include <celestial_tracker/CelestialTracker.h>

/**
** Define the CelestialTracker constructor
*/
CelestialTracker::CelestialTracker()
{

  // Initialize the telescope_position_text_publisher
  telescope_position_text_publisher_ = n_.advertise<telescope_msgs::EquatorialCoordinatesText>("telescope_position/text", 1, true);

  // Initialize the telescope_position_rad_publisher
  telescope_position_rad_publisher_ = n_.advertise<telescope_msgs::EquatorialCoordinates>("telescope_position/rad", 1, true);

  // Initialize the telescope_command_subscriber
  telescope_position_command_subscriber_ = n_.subscribe("telescope_position_command/rad", 1, &CelestialTracker::telescopePositionCommandCallback, this);

  // Initialize the change_tracking_mode_service_server
  change_tracking_mode_service_server_ = n_.advertiseService("change_tracking_mode", &CelestialTracker::changeTrackingModeServiceCallback, this);

  // Initialize the current_tracking_mode
  current_tracking_mode_ = telescope_msgs::ChangeTrackingMode::Request::NONE;

}


/**
** Define the CelestialTracker initialize function
*/
bool CelestialTracker::initialize(ros::NodeHandle &controler_pn)
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
** Define the CelestialTracker loopOnce function
*/
void CelestialTracker::loopOnce()
{
  ;;
}


/**
** Define the CelestialTracker telescopePositionCommandCallback function
*/
void CelestialTracker::telescopePositionCommandCallback(const telescope_msgs::EquatorialCoordinates &telescope_position_command_data)
{
  telescope_position_command_ = telescope_position_command_data;
}


/**
** Define the CelestialTracker changeTrackingModeServiceCallback function
*/
bool CelestialTracker::changeTrackingModeServiceCallback(telescope_msgs::ChangeTrackingModeRequest& req, telescope_msgs::ChangeTrackingModeResponse& res)
{
  // Switch from req mode
  switch (req.mode)
  {
    // None mode
    case telescope_msgs::ChangeTrackingMode::Request::NONE:
      // Change current_tracking_mode
      current_tracking_mode_ = telescope_msgs::ChangeTrackingMode::Request::NONE;
      // Set response to 0 (no error)
      res.result = 0;
      break;

    default:
      // Set response to -1 (unknow mode)
      res.result = -1;
      break;
  }
  // Return true (no error while handling mode change)
  // Error in mode value handled by res.result
  return true;
}
