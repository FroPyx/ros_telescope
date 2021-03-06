// Include libraries
#include <pluginlib/class_list_macros.h>
#include <guidance_plugins/manual_guidance.h>


// Define the guidance_mode namespace
namespace guidance_mode
{

/**
** Define ManualGuidance initialize function
*/
bool ManualGuidance::initialize(const std::string &guidance_type, const std::string& params_json)
{

  // Parameters :
  // - None

  // Set guidance type
  guidance_type_ = "ManualGuidance";

  // Check that guidance type is correct
  if (guidance_type != guidance_type_)
  {
    // If guidance type is wrong
    // Return false (failed to initialize guidance)
    return false;
  }

  // Decode parameters
  if (!decodeParameters(params_json))
  {
    // If parameters wrong
    // Return false (failed to initialize guidance)
    return false;
  }

  // Return true (initialization of action succeed)
  return true;
}


/**
* Define ManualGuidance executeAction function
*/
void ManualGuidance::executeAction(const actionlib::SimpleActionServer<mount_msgs::GuidanceAction>::GoalConstPtr &goal)
{

  // Define the manual guidance loop rate
  ros::Rate r(50);

  // While end of manual guidance not requested
  while (!stop_manual_guidance_)
  {

    // Update msgs
    ros::spinOnce();

    // Check time for cmd_vel
    if (ros::Time::now() - time_since_manual_guidance_cmd_ > ros::Duration(1.0))
    {
      // If timeout for command reset command
      equatorial_command_velocity_.ra = 0.0;
      equatorial_command_velocity_.dec = 0.0;
    }

    // Check time for UUID
    if (ros::Time::now() - time_since_manual_guidance_cmd_ > ros::Duration(15.0))
    {
      // If timeout for uuid connectivity
      result_.result = -1;
      result_.details = "{\"status\": \"Connectivity timed out\"}";
      action_server_.setAborted(result_);

      // Set uuid to none
      uuid_.data = "None";
      uuid_publisher_.publish(uuid_);

      return;
    }

    // Update guidance command coordinates
    updateGuidanceCommandCoordinates();

    // Publish guidance command
    publishGuidanceCommand();

    // Sleep
    r.sleep();
  }


  // Publish Result
  result_.result = 0;
  result_.details = "Guidance finished";
  action_server_.setSucceeded(result_);

}


/**
** Define ManualGuidance decodeParameters function
*/
bool ManualGuidance::decodeParameters(const std::string& parameters_json)
{
  try
  {
    // Define the parameter json
    nlohmann::json parameters = nlohmann::json::parse(parameters_json);

  }
  catch (...)
  {
    return false;
  }

  // If parameters decoded succesfully
  return true;
}



} // namespace action_plugin

PLUGINLIB_EXPORT_CLASS(guidance_mode::ManualGuidance, guidance_mode::BaseGuidance)
