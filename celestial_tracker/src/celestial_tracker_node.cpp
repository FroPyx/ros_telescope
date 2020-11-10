// Include librairies
#include <celestial_tracker/CelestialTracker.h>


int main(int argc, char** argv)
{

  // Initialize node
  ros::init(argc, argv, "celestial_tracker");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  // Initialize loop rate
  ros::Rate loop_rate(20);

  // Define celestial_tracker
  CelestialTracker celestial_tracker;

  // Initialize celestial_tracker
  if (!celestial_tracker.initialize(private_node))
  {
    // End node if initialization failed
    return -1;
  }


  // Until shutdown is requested
  while (ros::ok())
  {

    // Update ros messages/services
    ros::spinOnce();

    // Execute celestial_tracker loopOnce
    celestial_tracker.loopOnce();

    // Sleep
    loop_rate.sleep();

   }

  return 0;
}
