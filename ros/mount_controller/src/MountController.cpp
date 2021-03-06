// Include libraries
#include <MountController.h>


/**
** Define the MountController constructor
**/
MountController::MountController()
{

  // Initialize the mount_ra_motor_position_command_subscriber
  mount_ra_position_command_subscriber_ = node_handler_.subscribe("mount_ra_position_command", 1, &MountController::mountRaPositionCommandCallback, this);

  // Initialize the mount_dec_motor_position_command_subscriber
  mount_dec_position_command_subscriber_ = node_handler_.subscribe("mount_dec_position_command", 1, &MountController::mountDecPositionCommandCallback, this);


  // Initialize the mount_ra_motor_position_command_subscriber
  mount_ra_position_command_subscriber_ = node_handler_.subscribe("mount_ra_motor_position", 1, &MountController::mountRaMotorPositionCallback, this);

  // Initialize the mount_dec_motor_position_command_subscriber
  mount_dec_position_command_subscriber_ = node_handler_.subscribe("mount_dec_motor_position", 1, &MountController::mountDecMotorPositionCallback, this);


  // Initialize the mount_position_rad_publisher
  mount_position_rad_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position/rad", 1, true);

  // Initialize the mount_position_ha_publisher
  mount_position_ha_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position/ha", 1, true);

  // Initialize the mount_position_text_publisher
  mount_position_text_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position/text", 1, true);


  // Initialize the mount_position_command_rad_publisher
  mount_position_command_rad_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position_command/rad", 1, true);

  // Initialize the mount_position_command_ha_publisher
  mount_position_command_ha_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position_command/ha", 1, true);

  // Initialize the mount_position_command_text_publisher
  mount_position_command_text_publisher_ = node_handler_.advertise<mount_msgs::CelestialCoordinates>("mout_position_command/text", 1, true);


  // Initialize the mount_ra_motor_position_command_publisher
  mount_ra_motor_position_command_publisher_ = node_handler_.advertise<std_msgs::Int32>("mout_ra_motor_position_command", 1, true);

  // Initialize the mount_dec_motor_position_command_publisher
  mount_dec_motor_position_command_publisher_ = node_handler_.advertise<std_msgs::Int32>("mout_dec_motor_position_command", 1, true);

}


/**
** Define the MountController constructor
**/
bool MountController::init(const ros::NodeHandle& private_node_handler)
{

  // Get all parameters

  // Get mount_ra_motor_step_per_revolution param
  int mount_ra_motor_step_per_revolution;
  if (!private_node_handler.getParam("mount_ra_motor_step_per_revolution", mount_ra_motor_step_per_revolution))
  {
    return false;
  }

  // Get mount_dec_motor_step_per_revolution param
  int mount_dec_motor_step_per_revolution;
  if (!private_node_handler.getParam("mount_dec_motor_step_per_revolution", mount_dec_motor_step_per_revolution))
  {
    return false;
  }

  // Get mount_ra_motor_microstep param
  int mount_ra_motor_microstep;
  if (!private_node_handler.getParam("mount_ra_motor_microstep", mount_ra_motor_microstep))
  {
    return false;
  }

  // Get mount_dec_motor_microstep param
  int mount_dec_motor_microstep;
  if (!private_node_handler.getParam("mount_dec_motor_microstep", mount_dec_motor_microstep))
  {
    return false;
  }

  // Get mount_ra_gear_ratio param
  int mount_ra_gear_ratio;
  if (!private_node_handler.getParam("mount_ra_gear_ratio", mount_ra_gear_ratio))
  {
    return false;
  }

  // Get mount_dec_gear_ratio param
  int mount_dec_gear_ratio;
  if (!private_node_handler.getParam("mount_dec_gear_ratio", mount_dec_gear_ratio))
  {
    return false;
  }

  // Get mount_ra_worm_gear_ratio param
  int mount_ra_worm_gear_ratio;
  if (!private_node_handler.getParam("mount_ra_worm_gear_ratio", mount_ra_worm_gear_ratio))
  {
    return false;
  }

  // Get mount_dec_worm_gear_ratio param
  int mount_dec_worm_gear_ratio;
  if (!private_node_handler.getParam("mount_dec_worm_gear_ratio", mount_dec_worm_gear_ratio))
  {
    return false;
  }

  // Calculate rad_to_mount_ra_motor_ratio
  rad_to_mount_ra_motor_ratio_ = mount_ra_motor_step_per_revolution*mount_ra_motor_microstep*mount_ra_gear_ratio*mount_ra_worm_gear_ratio/(2*M_PI);

  // Calculate rad_to_mount_dec_motor_ratio
  rad_to_mount_dec_motor_ratio_ = mount_dec_motor_step_per_revolution*mount_dec_motor_microstep*mount_dec_gear_ratio*mount_dec_worm_gear_ratio/(2*M_PI);

  // If all parameters found => Return true
  return true;
}


/**
** Define the MountController loopOnce function
**/
void MountController::loopOnce()
{

  // Publish mount position
  publishMountPosition();

  // Publish mount position
  publishMountCommandPosition();

  // Publish mount motors command position
  publishMountMotorsCommandPosition();

}


/**
** Define the MountController publishMountPosition function
**/
void MountController::publishMountPosition()
{

  // Publish mount position (RAD)
  mount_position_rad_publisher_.publish(mount_position_rad_);

  // Publish mount position (Hour Angle)
  mount_position_ha_publisher_.publish(mount_position_ha_);

  // Publish mount position (Text)
  mount_position_ha_publisher_.publish(mount_position_text_);

}


/**
** Define the MountController publishMountPosition function
**/
void MountController::publishMountCommandPosition()
{

  // Publish mount command position (RAD)
  mount_position_command_rad_publisher_.publish(mount_position_rad_);

  // Publish mount command position (Hour Angle)
  mount_position_command_ha_publisher_.publish(mount_position_command_ha_);

  // Publish mount command position (Text)
  mount_position_command_ha_publisher_.publish(mount_position_command_text_);

}


/**
** Define the MountController publishMountMotorsCommandPosition function
**/
void MountController::publishMountMotorsCommandPosition()
{

  // Define the ra_motor_command_position
  std_msgs::Int32 mount_ra_motor_command_position;

  // Define the ra_motor_command_position
  std_msgs::Int32 mount_dec_motor_command_position;

  // Convert mount ra command position to motor position
  mount_ra_motor_command_position.data = static_cast<int32_t>(mount_position_command_rad_.ra*rad_to_mount_ra_motor_ratio_);

  // Convert mount ra command position to motor position
  mount_dec_motor_command_position.data = static_cast<int32_t>(mount_position_command_rad_.dec*rad_to_mount_dec_motor_ratio_);

  // Publish mount ra motor command position
  mount_ra_motor_position_command_publisher_.publish(mount_ra_motor_command_position);

  // Publish mount dec motor command position
  mount_dec_motor_position_command_publisher_.publish(mount_dec_motor_command_position);

}


/**
** Define the MountController mountRaPositionCommandCallback
**/
void MountController::mountRaPositionCommandCallback(const std_msgs::Float64& mount_ra_position_command_data)
{

  // Modify mount_position_command_rad
  mount_position_command_rad_.ra = mount_ra_position_command_data.data;

  // Modify mount_position_command_ha
  mount_position_command_ha_.ra = radToHourAngle(mount_ra_position_command_data.data);

  // Modify mount_position_command_text
  mount_position_command_text_.ra = radToHMS(mount_ra_position_command_data.data);

}


/**
** Define the MountController mountDecPositionCommandCallback
**/
void MountController::mountDecPositionCommandCallback(const std_msgs::Float64& mount_dec_position_command_data)
{

  // Modify mount_position_command_rad
  mount_position_command_rad_.dec = mount_dec_position_command_data.data;

  // Modify mount_position_command_ha
  mount_position_command_ha_.dec = radToDegree(mount_dec_position_command_data.data);

  // Modify mount_position_command_text
  mount_position_command_text_.dec = radToDMS(mount_dec_position_command_data.data);

}


/**
** Define the MountController mountRaMotorPositionCallback
**/
void MountController::mountRaMotorPositionCallback(const std_msgs::Int32& mount_ra_motor_position_data)
{

  // Modify mount_position_rad
  mount_position_rad_.ra = mount_ra_motor_position_data.data/rad_to_mount_ra_motor_ratio_;

  // Modify mount_position_rad
  mount_position_ha_.ra = radToHourAngle(mount_ra_motor_position_data.data/rad_to_mount_ra_motor_ratio_);

  // Modify mount_position_text
  mount_position_text_.ra = radToHMS(mount_ra_motor_position_data.data/rad_to_mount_ra_motor_ratio_);

}


/**
** Define the MountController mountDecMotorPositionCallback
**/
void MountController::mountDecMotorPositionCallback(const std_msgs::Int32& mount_dec_motor_position_data)
{

  // Modify mount_position_rad
  mount_position_rad_.dec = mount_dec_motor_position_data.data/rad_to_mount_dec_motor_ratio_;

  // Modify mount_position_rad
  mount_position_ha_.dec = radToDegree(mount_dec_motor_position_data.data/rad_to_mount_dec_motor_ratio_);

  // Modify mount_position_text
  mount_position_text_.dec = radToDMS(mount_dec_motor_position_data.data/rad_to_mount_dec_motor_ratio_);

}
