// Include libraries
#include <guidance_plugins/base_guidance.h>


// Define the guidance_mode namespace
namespace guidance_mode
{

/**
** Define BaseGuidance initialize function
*/
bool BaseGuidance::initialize(const std::string &guidance_type, const std::string& params_json)
{

  // Parameters :
  // None

  // Set guidance type
  guidance_type_ = "BaseAction";

  // Check that guidance type is correct
  if (guidance_type != guidance_type_)
  {
    // If guidance type is wrong
    // Return false (failed to initialize action)
    return false;
  }

  // Decode parameters
  if (!decodeParameters(params_json))
  {
    // If parameters wrong
    // Return false (failed to initialize action)
    return false;
  }

  // Return true (initialization of action succeed)
  return true;
}


/**
** Define BaseGuidance decodeParameters function
*/
bool BaseGuidance::decodeParameters(const std::string& parameters_json)
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


/**
* Define BaseGuidance executeAction function
*/
void BaseGuidance::executeAction(const actionlib::SimpleActionServer<mount_msgs::GuidanceAction>::GoalConstPtr &goal)
{

  // Sleep 5 second for the example
  ros::Rate r(0.2);
  r.sleep();

}

} // namespace action_base
