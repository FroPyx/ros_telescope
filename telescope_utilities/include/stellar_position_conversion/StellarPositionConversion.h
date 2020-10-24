#ifndef STELLAR_POSITION_CONVERSION_H
#define STELLAR_POSITION_CONVERSION_H 1

// Include librairies
#include <cmath>
#include <iomanip>
#include <vector>

/**
** Define the radToDegree function
*/
double radToDegree(const double& rad)
{
  return rad*180/M_PI;
}


/**
** Define the hourAngleToRad function
*/
double hourAngleToRad(const double& hour_angle)
{
  return hour_angle*M_PI/12.0;
}


/**
** Define the hourAngleToDegree function
*/
double hourAngleToDegree(const double& hour_angle)
{
  return radToDegree(hourAngleToRad(hour_angle));
}


/**
** Define the degreeToRad function
*/
double degreeToRad(const double& degree)
{
  return degree*M_PI/180.0;
}



/**
** Define the radToHourAngle function
*/
double radToHourAngle(const double& rad)
{
  return rad*12/M_PI;
}


/**
** Define the degreeToHourAngle function
*/
double degreeToHourAngle(const double& degree)
{
  return radToHourAngle(degreeToRad(degree));
}



/**
** Define the degreeToDMS function
*/
std::string degreeToDMS(const double& degree)
{

  // Define the degree
  double degree_angle = degree;

  // Ensure the value will fall within the primary range [-180.0..+180.0]
  while (degree_angle < -180.0)
  {
    degree_angle += 360.0;
  }
  while (degree_angle > 180.0)
  {
    degree_angle -= 360.0;
  }

  // Define the absolute value of degree
  double abs_degree = std::abs(degree_angle);

  // Define the dms_degrees
  int dms_degrees = static_cast<int>(std::floor(abs_degree));

  // Define the dms_minutes
  int dms_minutes = static_cast<int>(std::floor((abs_degree - dms_degrees)*60));

  // Define the dms_second
  double dms_seconds = (abs_degree - dms_degrees - dms_minutes/60.0)*3600;


  // Define the dms_string
  std::string dms = "";

  // If degree is negative
  if (degree_angle < 0.0)
  {
    // Add negative symbol
    dms += "-";
  }

  // Add degree
  if (dms_degrees < 10)
  {
    dms += "0";
  }
  dms += std::to_string(dms_degrees) + "°";

  // Add minutes
  if (dms_minutes < 10)
  {
    dms += "0";
  }
  dms += std::to_string(dms_minutes) + "'";

  // Create an output string stream
  std::ostringstream stream_object;

  // Set fixed point notation
  stream_object << std::fixed;

  // Set precision to 2 digits
  stream_object << std::setprecision(2);

  // Add dms_seconds to stream
  stream_object << dms_seconds;

  // Add seconds
  if (dms_seconds < 10)
  {
    dms += "0";
  }
  dms += stream_object.str() + "\"";

  // Return DMS
  return dms;
}


/**
** Define the radToDMS function
*/
std::string radToDMS(const double& rad)
{
  return degreeToDMS(radToDegree(rad));
}


/**
** Define the hourAngleToDMS function
*/
std::string hourAngleToDMS(const double& hour_angle)
{
  return degreeToDMS(hourAngleToDegree(hour_angle));
}



/**
** Define the hourAngleToHMS function
*/
std::string hourAngleToHMS(const double& hour_angle)
{

  // Define the hour_angle for calculation
  double hour_angle_calculation = hour_angle;

  // Ensure the value will fall within the primary range [-24.0..+24.0]
  while (hour_angle_calculation < -24.0)
  {
    hour_angle_calculation += 24.0;
  }
  while (hour_angle_calculation > 24.0)
  {
    hour_angle_calculation -= 24.0;
  }

  // Define the absolute value of hour angle
  double abs_hour_angle = std::abs(hour_angle_calculation);

  // Define the hms_hours
  int hms_hours = static_cast<int>(std::floor(abs_hour_angle));

  // Define the hms_minutes
  int hms_minutes = static_cast<int>(std::floor((abs_hour_angle - hms_hours)*60));

  // Define the hms_second
  double hms_seconds = (abs_hour_angle - hms_hours - hms_minutes/60.0)*3600;


  // Define the dms_string
  std::string hms = "";

  // If hour angle is negative
  if (hour_angle_calculation < 0.0)
  {
    // Add negative symbol
    hms += "-";
  }

  // Add degree
  if (hms_hours < 10)
  {
    hms += "0";
  }
  hms += std::to_string(hms_hours) + "h";

  // Add minutes
  if (hms_minutes < 10)
  {
    hms += "0";
  }
  hms += std::to_string(hms_minutes) + "m";

  // Create an output string stream
  std::ostringstream stream_object;

  // Set fixed point notation
  stream_object << std::fixed;

  // Set precision to 2 digits
  stream_object << std::setprecision(2);

  // Add dms_seconds to stream
  stream_object << hms_seconds;

  // Add seconds
  if (hms_seconds < 10)
  {
    hms += "0";
  }
  hms += stream_object.str() + "s";

  // Return DMS
  return hms;
}



/**
** Define the degreeToHMS function
*/
std::string degreeToHMS(const double& degree)
{
  return hourAngleToHMS(degreeToHourAngle(degree));
}



/**
** Define the radToHMS function
*/
std::string radToHMS(const double& rad)
{
  return hourAngleToHMS(radToHourAngle(rad));
}


/**
** Define the equatorialCoordinatesRadToDegree function
*/
std::vector<double> equatorialCoordinatesRadToDegree(const std::vector<double>& equatorial_coordinates_rad)
{
  // Initialize the equatorial_coordinates_degree
  std::vector<double> equatorial_coordinates_degree;

  // Add RA coordinate
  equatorial_coordinates_degree.push_back(radToDegree(equatorial_coordinates_rad[0]));

  // Add DEC coordinate
  equatorial_coordinates_degree.push_back(radToDegree(equatorial_coordinates_rad[1]));

  // Return equatorial_coordinates_degree
  return equatorial_coordinates_degree;
}

#endif // STELLAR_POSITION_CONVERSION_H
