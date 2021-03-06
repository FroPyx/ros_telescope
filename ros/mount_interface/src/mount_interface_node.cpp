// include librairies
#include <MountInterface.h>

/*
** Define the main function
*/
int main(int argc, char** argv)
{

  // Create ROS node
  ros::init(argc, argv, "mount_interface");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");


  // Init control rate
  double control_rate;
  // Get wheel radius multipler param
  if (!pn.getParam("control_rate", control_rate))
  {
    return 1;
  }
  ros::Rate loop_rate(control_rate);

  // Init the motor driver
  MountInterface mount_interface;
  if (!mount_interface.init(pn))
  {
    return 1;
  }

  // Until shutdown is requested
  while (ros::ok())
  {
    // Update ros messages
    ros::spinOnce();

    // Update mount interface
    mount_interface.loopOnce();

    // Sleep rate
    loop_rate.sleep();
  }

  return 0;
}


  

