// Include librairies
#include <AccelStepper.h>
#include <ArduinoJson.h>

// Define stepper motor connections
#define ra_direction_pin 5
#define ra_step_pin 2
#define dec_direction_pin 6
#define dec_step_pin 3
#define enable_pin 8

// Define the motorInterfaceType (driver)
#define motorInterfaceType 1

// Define the json capacity
const int json_capacity_ = 2*JSON_OBJECT_SIZE(20);

// Define the JsonDocument
StaticJsonDocument<json_capacity_> json_doc_;

// Declare ra_motor
AccelStepper ra_motor_ = AccelStepper(motorInterfaceType, ra_step_pin, ra_direction_pin, 0, 0, false);

// Declare dec_motor
AccelStepper dec_motor_ = AccelStepper(motorInterfaceType, dec_step_pin, dec_direction_pin, 0, 0, false);

// Declare motors_enabled_status
bool motors_enabled_status_ = false;

/** 
 * Define setup function (initialize serial and output)
 */
void setup()
{

  // Set ra_motor speed and acceleration
  ra_motor_.setMaxSpeed(200);
  ra_motor_.setSpeed(200);
  ra_motor_.setAcceleration(50);

  // Set ra_motor enable pin (controls dec motor too)
  ra_motor_.setEnablePin(enable_pin);
  ra_motor_.setPinsInverted(false, false, true);
  
  // Set dec_motor speed and acceleration
  dec_motor_.setMaxSpeed(4500);
  dec_motor_.setSpeed(4500);
  dec_motor_.setAcceleration(500);

  // Initialize serial interface with the Ros telescope controller
  Serial.begin(115200);
  
}


/** 
 * Define loop function
 */
void loop()
{
  // Get Serial command
  getSerialCommand();

  // Set motors position
  setMotorsPosition();
}


/** 
 * Define getSerialCommand function
 */
void getSerialCommand()
{
  // Check if serial datas are availaible
  if (Serial.available())
  {
    // Get serial command 
    String serial_command = Serial.readStringUntil('\n');

    // Deserialize command
    DeserializationError err = deserializeJson(json_doc_, serial_command);

    // If an error occured 
    if (err != DeserializationError::Ok)
    {
      //Serial.flush();
      // End of function
      return;
    }
    
    // Witch command
    switch ((int8_t)json_doc_["cmd"])
    {
      // Command 0 (set ra_position)
      case 0:
        ra_motor_.setCurrentPosition((long)json_doc_["value"]);
        break;
        
      // Command 1 (set dec_position)
      case 1:
        dec_motor_.setCurrentPosition((long)json_doc_["value"]);
        break;
        
      // Command 2 (set ra_position_command)
      case 2:
        ra_motor_.moveTo((long)json_doc_["value"]);
        break;
        
      // Command 3 (set dec_position_command)
      case 3:
        dec_motor_.moveTo((long)json_doc_["value"]);
        break;
        
      // Command 4 (enable or disable motors)
      case 4:
        // If enable motors
        if((int)json_doc_["value"] == 1)
        {
          // Enable motors
          ra_motor_.enableOutputs();
          dec_motor_.enableOutputs();
          motors_enabled_status_ = true;
        }
        else
        {
          // Disable motors
          ra_motor_.disableOutputs();
          motors_enabled_status_ = false;

          // Reset motor speed and position to go
          ra_motor_.setAcceleration(10000);
          dec_motor_.setAcceleration(10000);
          ra_motor_.moveTo(ra_motor_.currentPosition());
          dec_motor_.moveTo(dec_motor_.currentPosition());
          while (ra_motor_.isRunning() || dec_motor_.isRunning())
          {
            ra_motor_.run();
            dec_motor_.run();
          }
          
          // Reset motors acceleration
          ra_motor_.setAcceleration(500);
          dec_motor_.setAcceleration(500);
        }
        break;
    }
  }
}


/** 
 * Define setMotorsPosition function
 */
void setMotorsPosition()
{
  if (motors_enabled_status_)
  {
    // Control ra motor position
    ra_motor_.run();

    // Control dec motor position
    dec_motor_.run();
  }  
}
