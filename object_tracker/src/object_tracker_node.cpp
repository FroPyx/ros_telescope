// Include librairies
#include <object_tracker/ObjectTracker.h>


int main(int argc, char** argv)
{

  // Initialize node
  ros::init(argc, argv, "object_tracker");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  // Initialize loop rate
  ros::Rate loop_rate(20);

  // Define object_tracker
  ObjectTracker object_tracker;

  // Initialize object_tracker
  if (!object_tracker.initialize(private_node))
  {
    // End node if initialization failed
    return -1;
  }


  // Until shutdown is requested
  while (ros::ok())
  {

    // Update ros messages/services
    ros::spinOnce();

    // Execute local_sidereal_time loopOnce
    object_tracker.loopOnce();

    // Sleep
    loop_rate.sleep();

   }

  return 0;
}
