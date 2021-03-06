#ifndef PLUGINLIB_MANUAL_GUIDANCE_H_
#define PLUGINLIB_MANUAL_GUIDANCE_H_ 1

// Include libraries
#include <guidance_plugins/base_guidance.h>
#include <std_msgs/String.h>
#include <mount_msgs/EquatorialCoordinates.h>


// Define the guidance_mode namespace
namespace guidance_mode
{

/**
** Define ManualGuidance constructor
*/
class ManualGuidance : public guidance_mode::BaseGuidance
{

public:

  // Define the constructor
  ManualGuidance(): action_server_(n_, "guidance/ManualGuidance", boost::bind(&ManualGuidance::executeAction, this, _1), false)
  {
    // Initialize the uuid publisher
    uuid_publisher_ = n_.advertise<std_msgs::String>("manual_steering_uuid", 1, true);

    // Initialize the uuid
    uuid_.data = "None";
  }

  // Define the initialize function
  bool initialize(const std::string &guidance_type, const std::string& params_json);

  // Define the start action server function
  virtual void startActionServer(){action_server_.start();}

  // Define the action server
  actionlib::SimpleActionServer<mount_msgs::GuidanceAction> action_server_;

protected:

  // Define the execute action function
  void executeAction(const actionlib::SimpleActionServer<mount_msgs::GuidanceAction>::GoalConstPtr &goal);

  // Define the decodeParameters function
  bool decodeParameters(const std::string& parameters_json);


private:


  // Define the uuid publisher
  ros::Publisher uuid_publisher_;


  // Define time since time_since_manual_guidance_cmd was received
  ros::Time time_since_manual_guidance_cmd_;

  // Define the stop_manual_steering variable
  bool stop_manual_guidance_;

  // Define the uuid
  std_msgs::String uuid_;

  // Define the equatorial_command_velocity
  mount_msgs::EquatorialCoordinates equatorial_command_velocity_;


  // Define the updateGuidanceCommandCoordinates
  void updateGuidanceCommandCoordinates();

  // Define the publishGuidanceCommand function
  void publishGuidanceCommand();

  // Define the mountCommandVelocityCallback function
  void mountCommandVelocityCallback();

};


} // namespace guidance_mode

#endif
