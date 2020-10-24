#include <object_tracker/ObjectTracker.h>

/**
** Define the ObjectTracker constructor
*/
ObjectTracker::ObjectTracker()
{

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
  ;;
}


