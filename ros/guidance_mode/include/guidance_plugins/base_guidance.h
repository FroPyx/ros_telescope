#ifndef PLUGINLIB_BASE_GUIDANCE_H_
#define PLUGINLIB_BASE_GUIDANCE_H_ 1

// Include libraries
#include <ros/ros.h>
#include <mount_msgs/GuidanceAction.h>
#include <actionlib/server/simple_action_server.h>
#include <nlohmann/json.hpp>


// Define the guidance_mode namespace
namespace guidance_mode
{

/**
** Define BaseGuidance constructor
*/
class BaseGuidance
{

// Public functions and members


public:

  // Rename boost::shared_ptr<guidance_mode::BaseGuidance> as Ptr
  using Ptr = boost::shared_ptr<guidance_mode::BaseGuidance>;

  // Define the constructor
  BaseGuidance(){}

  // Define the initialize function
  virtual bool initialize(const std::string &guidance_type, const std::string& params_json);

  // Define the start action server function
  virtual void startActionServer(){}


// Protected members
protected:

  // Define the execute action function
  virtual void executeAction(const actionlib::SimpleActionServer<mount_msgs::GuidanceAction>::GoalConstPtr &goal);

  // Define the decodeParameters function
  bool decodeParameters(const std::string& parameters_json);

  // Define the node handler
  ros::NodeHandle n_;

  // Define the action type
  std::string guidance_type_;

  // Define the action topic
  std::string guidance_topic_;

  // Define guidance feedback and result
  mount_msgs::GuidanceFeedback feedback_;
  mount_msgs::GuidanceResult result_;

};


} // namespace guidance_mode


#endif
