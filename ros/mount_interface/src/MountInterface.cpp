// Include libraries
#include <MountInterface.h>


/**
** Define the MountInterface constructor
**/
MountInterface::MountInterface()
{

  // Initialize the mount_ra_motor_position_command_subscriber
  mount_ra_motor_position_command_subscriber_ = node_handler_.subscribe("mount_ra_motor_position_command", 1, &MountInterface::mountRaMotorPositionCommandCallback, this);

  // Initialize the mount_dec_motor_position_command_subscriber
  mount_dec_motor_position_command_subscriber_ = node_handler_.subscribe("mount_dec_motor_position_command", 1, &MountInterface::mountDecMotorPositionCommandCallback, this);

  // Initialize the mount_reset_motors_position_command_subscriber
  mount_reset_motors_position_command_subscriber_ = node_handler_.subscribe("mount_reset_motors_position_command", 1, &MountInterface::mountResetMotorsPositionCommandCallback, this);

  // Initialize the mount_enable_motors_power_command_subscriber
  mount_enable_motors_power_command_subscriber_ = node_handler_.subscribe("mount_enable_motors_power_command", 1, &MountInterface::mountEnableMotorsPowerCommandCallback, this);

  // Initialize the mount_ra_motor_position publisher
  mount_ra_motor_position_publisher_ = node_handler_.advertise<std_msgs::Int32>("mount_ra_motor_position", 1, true);

  // Initialize the mount_dec_motor_position publisher
  mount_dec_motor_position_publisher_ = node_handler_.advertise<std_msgs::Int32>("mount_dec_motor_position", 1, true);

  // Initialize the mount_enable_motors_power_pusblisher publisher
  mount_enable_motors_power_publisher_ = node_handler_.advertise<std_msgs::Bool>("mount_enable_motors_power", 1, true);
}


/**
** Define the MountInterface constructor
**/
bool MountInterface::init(const ros::NodeHandle& private_node_handler)
{

  // Get all parameters

  // Get serial port parameter
  if (!private_node_handler.getParam("serial_port", serial_port_))
  {
    // If parameter not found => Return false
    return false;
  }

  // Get serial_baudrate parameter
  if (!private_node_handler.getParam("serial_baudrate", serial_baudrate_))
  {
    // If parameter not found => Return false
    return false;
  }

  // Get serial_timeout param
  if (!private_node_handler.getParam("serial_timeout", serial_timeout_))
  {
    return false;
  }

  // If all parameters found => Return true
  return true;
}


/**
** Define the MountInterface loopOnce function
**/
void MountInterface::loopOnce()
{
  // Check if device is connected
  if (!serial_interface_.isOpen())
  {
    // If not connected

    // Configure serial port
    serial_interface_.setPort(serial_port_);
    serial_interface_.setBaudrate(static_cast<uint32_t>(serial_baudrate_));
    serial_interface_.setBytesize(serial::eightbits);
    serial_interface_.setParity(serial::parity_none);
    serial_interface_.setStopbits(serial::stopbits_one);
    //serial_interface_.setTimeout(serial::Timeout::max(), 100, 0, 100, 0);

    // Try to connect
    try
    {
      // Connect
      serial_interface_.open();

      // Wait 1 second until Serial inteface is up
      ros::Duration(1.0).sleep();
    }
    catch(...)
    {
      // If error while connecting => Do nothing
      ;;
    }
    // End of function if not connected initialy
    return;
  }

  // If device is connected

  // Get serial data
  getSerialData();

  // Update serial data
  updateSerialData();

  // Send serial command
  sendSerialCommand();
}


/**
** Define the MountInterface getSerialData
**/
void MountInterface::getSerialData()
{
  // Check if serial data are available
  if (serial_interface_.available())
  {
    // If serial data are available

    // Define the new serial data
    std::vector<std::string> new_serial_data;

    // Read data until EOL
    new_serial_data = serial_interface_.readlines();

    // Add new serial data to serial data
    serial_data_.insert(serial_data_.end(), new_serial_data.begin(), new_serial_data.end());
  }
}


/**
** Define the MountInterface updateSerialData
**/
void MountInterface::updateSerialData()
{
  // For each serial data
  for (std::string& data : serial_data_)
  {
    // Try do deserialize the data
    try
    {
      // Deserialize the data
      nlohmann::json json_data = nlohmann::json::parse(data);

      // Check witch data
      if (json_data["data"] == "ra_motor_position")
      {
        // If data is ra_motor_position

        // Define ra_motor_position message
        std_msgs::Int32 ra_motor_position;
        ra_motor_position.data = json_data["value"];

        // Publish ra_motor_position
        mount_ra_motor_position_publisher_.publish(ra_motor_position);
      }
      else if (json_data["data"] == "dec_motor_position")
      {
        // If data is dec_motor_position

        // Define dec_motor_position message
        std_msgs::Int32 dec_motor_position;
        dec_motor_position.data = json_data["value"];

        // Publish dec_motor_position
        mount_dec_motor_position_publisher_.publish(dec_motor_position);
      }
      else if (json_data["data"] == "enable_motors_power")
      {
        // If data is enable_motors_power

        // Define dec_motor_position message
        std_msgs::Bool enable_motors_power;
        enable_motors_power.data = json_data["value"];

        // Publish enable_motors_power
        mount_enable_motors_power_publisher_.publish(enable_motors_power);
      }

    }
    catch (...)
    {
      // In case of error do nothing
      ;;
    }
  }

  // Clear serial data
  serial_data_.clear();

}


/**
** Define the MountInterface sendSerialCommand
**/
void MountInterface::sendSerialCommand()
{
  // For each serial command
  for (std::string& command : serial_command_)
  {
    // Define the number of byte written
    size_t bytes_written;

    // Send command to serial interface
    bytes_written = serial_interface_.write(command);

    // Check if number of bytes written are the same as the command size
    if (bytes_written != command.size())
    {
      // If number of bytes written is different of the size of command

      // Try to close the serial_interface
      try
      {
        serial_interface_.close();
      }
      catch (...)
      {
        ;;
      }
    }
  }

  // Clear serial command
  serial_command_.clear();
}


/**
** Define the MountInterface mountRaMotorPositionCommandCallback
**/
void MountInterface::mountRaMotorPositionCommandCallback(const std_msgs::Int32& ra_motor_position_command_data)
{
  // Define the command
  std::string command = "<{\"command\":\"ra_motor_position_command\",";
  command += "\"value\":" + std::to_string(ra_motor_position_command_data.data) + "}>";

  // Add command to serial_command
  serial_command_.emplace_back(command);
}


/**
** Define the MountInterface mountDecMotorPositionCommandCallback
**/
void MountInterface::mountDecMotorPositionCommandCallback(const std_msgs::Int32& dec_motor_position_command_data)
{
  // Define the command
  std::string command = "<{\"command\":\"dec_motor_position_command\",";
  command += "\"value\":" + std::to_string(dec_motor_position_command_data.data) + "}>";

  // Add command to serial_command
  serial_command_.emplace_back(command);
}


/**
** Define the MountInterface mountResetMotorsPositionCommandCallback
**/
void MountInterface::mountResetMotorsPositionCommandCallback(const std_msgs::Empty& reset_motors_position_command_data)
{
  // Define the command
  std::string command = "<{\"command\":\"reset_motors_position_command\"}>";

  // Add command to serial_command
  serial_command_.emplace_back(command);
}


/**
** Define the MountInterface mountResetMotorsPositionCommandCallback
**/
void MountInterface::mountEnableMotorsPowerCommandCallback(const std_msgs::Bool& enable_motors_power_command_data)
{
  // Define the command
  std::string command = "<{\"command\":\"enable_motors_power_command\",";
  command += "\"value\":" + std::to_string(enable_motors_power_command_data.data) + "}>";

  // Add command to serial_command
  serial_command_.emplace_back(command);
}
