// Include librairies
#include <local_sidereal_time/LocalSiderealTime.h>

int main(int argc, char** argv)
{

  // Initialize node
  ros::init(argc, argv, "local_sidereal_time");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  // Initialize loop rate
  ros::Rate loop_rate(100);

  // Define local_sidereal_time
  LocalSiderealTime local_sidereal_time;

  // Initialize local_sidereal_time
  if (!local_sidereal_time.initialize())
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
    local_sidereal_time.loopOnce();

    // Sleep
    loop_rate.sleep();

   }

  return 0;
}
