^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
All changes to this package will be documented in this file.
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2021-02-28)
----------------------
* Add updateSeialData function to publish information of the arduino shield
*
* Maintainer : Christophe Heymann


1.0.3 (2021-02-27)
----------------------
* Update serial command to be complient with the arduino shield to command motors
*
* Maintainer : Christophe Heymann


1.0.2 (2021-02-23)
----------------------
* Add nlohman/json library to MountInterface.h file
* Add std_msgs/Int32 library to MountInterface.h file
* Add std_msgs/Empty library to MountInterface.h file
*
* Modification of serial_baudrate member of MountInterface class to uint32_t type
* Add serial_timeout member to MountInterface class
* Add serial_data member to MountInterface class
* Add serial_command to MountInterface class
* Add mount_reset_motor_position_command_subscriber_ to MountInterface class
* Remplacement of mount_motors_position_command_subscriber to mount_ra_motor_position_command_subscriber and mount_dec_motor_position_command_subscriber
* Remplacement of mount_motors_position_publisher to mount_ra_motor_position_publisher and mount_dec_motor_position_publisher
*
*
* Add getSerialData function to MountInterface :
*   Get new serial_data from serial interface and insert it to serial_data
*
* Add sendSerialCommand function to MountInterface :
*   Send all serial_command to serial interface and clear serial_command
*
* Add mountRaMotorPositionCommandCallback function to MountInterface
*
* Add mountDecMotorPositionCommandCallback function to MountInterface
*
* Add mountResetMotorPositionCommandCallback function to MountInterface
*
* Modification of init function to MountInterface :
*   Add parameter private_node_handler
*   Get serial_timeout from private_node_handler
*
* Modification of loopOnce function to MountInterface :
*   Connect to serial interface
*   Call to getSerialData function
*
* Add std_msgs as dependency to package.xml and CMakeLists.txt
*
* Maintainer : Christophe Heymann


1.0.1 (2021-02-22)
----------------------
* Add serial_baudrate member to MountInterface class
* Add node handler member to MountInterface class
* Add mount_motors_position_command_subscriber member
* Add mount_motors_position_publisher member
*
* Modification of init function :
*   Add parameter private_node_handler
*   Get serial_port and serial_baudrate parameter from private_node_handler

* Maintainer : Christophe Heymann


## [1.0.0] 2021-02-21
----------------------
* Creation of the MountInterface.h and MountInterface.cpp files
* Add MountInterface class definition in MountInterface.h and MountInterface.cpp
* Add Contructor, Destructor, init and loopOnce functions to MountInterface class
* Add serial_port member to MountInterface class
* Add <serial/serial.h> as header to MountInterface.h
* Creation of package.xml file
* Creation of CMakeLists.txt file
*
* Maintainer : Christophe Heymann

