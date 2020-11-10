// Include librairies
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <telescope_msgs/MotorsStatusRaw.h>
#include <AccelStepper.h>


// Define stepper motor connections
#define ra_direction_pin_ 5
#define ra_step_pin_ 2
#define dec_direction_pin_ 6
#define dec_step_pin_ 3
#define enable_pin_ 8

// Define the motor_interface_type (driver)
#define motor_interface_type_ 1

// Define ra_motor
AccelStepper ra_motor_ = AccelStepper(motor_interface_type_, ra_step_pin_, ra_direction_pin_, 0, 0, false);

// Define dec_motor
AccelStepper dec_motor_ = AccelStepper(motor_interface_type_, dec_step_pin_, dec_direction_pin_, 0, 0, false);

// Define the ros node handler
ros::NodeHandle n_;

// Define the time_since_last_motors_status_publication
unsigned long time_since_last_motors_status_publication_;

// Define the motors_status_publication_timeout_
unsigned long motors_status_publication_timeout_;

// Define the motors status topic
telescope_msgs::MotorsStatusRaw motors_status_;


/** 
 * Define raMotorPositionCommandCallback function
 */
void raMotorPositionCommandCallback(const std_msgs::Int32& ra_motor_position_command_data)
{
  // Set the new ra motor position command
  ra_motor_.moveTo(ra_motor_position_command_data.data);  
}

/** 
 * Define decMotorPositionCommandCallback function
 */
void decMotorPositionCommandCallback(const std_msgs::Int32& dec_motor_position_command_data)
{
  // Set the new dec motor position command
  dec_motor_.moveTo(dec_motor_position_command_data.data);  
}

/** 
 * Define enableMotorsCommandCallback function
 */
void enableMotorsCommandCallback(const std_msgs::Bool& enable_motors_command_data)
{
  // Enable or disable motors (both motor are controled by the the ra_motor)
  if (enable_motors_command_data.data)
  {
    ra_motor_.enableOutputs();
    motors_status_.motors_enabled = true;
  }
  else
  {
    ra_motor_.disableOutputs();
    motors_status_.motors_enabled = false;
  }
}

// Define the ra_motor_command_subscriber
ros::Subscriber<std_msgs::Int32> ra_motor_command_subscriber_("ra_motor_command", &raMotorPositionCommandCallback);

// Define the dec_motor_command_subscriber
ros::Subscriber<std_msgs::Int32> dec_motor_command_subscriber_("dec_motor_command", &decMotorPositionCommandCallback);

// Define the enable_motors_command_subscriber
ros::Subscriber<std_msgs::Bool> enable_motors_command_subscriber_("enable_motors_command", &enableMotorsCommandCallback);


// Define the motors_status_publisher
ros::Publisher motors_status_publisher_("motor_status", &motors_status_);


/** 
 * Define setup function (initialize serial and output)
 */
void setup()
{
  // Initialize time
  time_since_last_motors_status_publication_ = millis();

  // Set motors_status_publication_timeout
  motors_status_publication_timeout_ = 200;

  // Initialize motors status
  motors_status_.motors_enabled = false;
  motors_status_.ra_motor_position = 0;
  motors_status_.dec_motor_position = 0;
  
  // Set ra_motor speed and acceleration
  ra_motor_.setMaxSpeed(200);
  ra_motor_.setSpeed(200);
  ra_motor_.setAcceleration(50);

  // Set ra_motor enable pin (controls dec motor too)
  ra_motor_.setEnablePin(enable_pin_);
  ra_motor_.setPinsInverted(false, false, true);
  
  // Set dec_motor speed and acceleration
  dec_motor_.setMaxSpeed(4500);
  dec_motor_.setSpeed(4500);
  dec_motor_.setAcceleration(500);
}


/** 
 * Define loop function
 */
void loop()
{
  // Update command
  n_.spinOnce();
  
  // Set motors position
  setMotorsPosition();

  // Publish motors status
  publishMotorsStatus();
}


/** 
 * Define setMotorsPosition function
 */
void setMotorsPosition()
{
  // If motors are enabled
  if (motors_status_.motors_enabled)
  {
    // Control ra motor position
    ra_motor_.run();

    // Control dec motor position
    dec_motor_.run();
  }  
}


/** 
 * Define publishMotorsStatus function
 */
void publishMotorsStatus()
{
  // Check if timeout for publication
  if (millis() - time_since_last_motors_status_publication_ > motors_status_publication_timeout_)
  {
    // Update motors status information
    motors_status_.ra_motor_position = ra_motor_.currentPosition();
    motors_status_.dec_motor_position = dec_motor_.currentPosition();

    // Publish motors_status
    motors_status_publisher_.publish(&motors_status_);

    // Update time since motors status publication
    time_since_last_motors_status_publication_ = millis();
  }
}
