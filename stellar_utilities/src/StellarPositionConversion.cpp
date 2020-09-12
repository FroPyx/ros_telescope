#include <stellar_position_conversion/StellarPositionConversion.h>

/**
** Define the radPositionFormatToHourAnglePositionFormat function
*/
std::vector<double> radPositionFormatToHourAnglePositionFormat(const std::vector<double>& rad_stellar_position)
{

  // Extract rad position
  double ra_rad_position = rad_stellar_position[0];
  double dec_rad_position = rad_stellar_position[1];

  // Convert ra_position to hour_angle

  // Initialize hour_angle position
  double ra_hour_angle_position = ra_rad_position*12.0/M_PI;
  double dec_angular_position = dec_rad_position*180.0/M_PI;

  // Return the converted values
  std::vector<double> hour_angle_stellar_position;
  hour_angle_stellar_position.push_back(ra_hour_angle_position);
  hour_angle_stellar_position.push_back(dec_angular_position);
  return hour_angle_stellar_position;

}


/**
** Define the radPositionFormatToHourAngleTextPositionFormat function
*/
std::vector<std::string> radPositionFormatToHourAngleTextPositionFormat(const std::vector<double>& rad_stellar_position)
{

  // Convert radFormat to HourAngleFormat
  std::vector<double> hour_angle_stellar_position = radPositionFormatToHourAnglePositionFormat(rad_stellar_position);


  // Extract hour_angle position
  double ra_hour_angle_position = hour_angle_stellar_position[0];
  double dec_hour_angle_position = hour_angle_stellar_position[1];


  // Convert hour angle to text format
  std::string ra_hour_angle_text_position = "";
  std::string dec_hour_angle_text_position = "";

  // Convert ra_hour_angle to text
  int ra_hour = static_cast<int>(ra_hour_angle_position);
  int ra_min = static_cast<int>((ra_hour_angle_position-ra_hour)*60);
  double ra_sec = (ra_hour_angle_position - ra_hour - ra_min/60.0)*3600;
  if (ra_hour < 0)
  {
    ra_hour_angle_text_position += "-";
  }
  if (abs(ra_hour) < 10)
  {
    ra_hour_angle_text_position += "0";
  }
  ra_hour_angle_text_position += std::to_string(abs(ra_hour)) + "h";
  if (ra_min < 10)
  {
    ra_hour_angle_text_position += "0";
  }
  ra_hour_angle_text_position += std::to_string(ra_min) + "m";
  if (ra_sec < 10)
  {
    ra_hour_angle_text_position += "0";
  }
  std::stringstream ss_ra_sec;
  ss_ra_sec << std::fixed << std::setprecision(2) << ra_sec;
  ra_hour_angle_text_position += ss_ra_sec.str() + "s";

  // Convert dec_angle to text
  int dec_deg = static_cast<int>(dec_hour_angle_position);
  int dec_min = static_cast<int>((dec_hour_angle_position-dec_deg)*60);
  double dec_sec = (dec_hour_angle_position - dec_deg - dec_min/60.0)*3600;
  dec_hour_angle_text_position += std::to_string(dec_deg) + "°";
  if (dec_min < 10)
  {
    dec_hour_angle_text_position += "0";
  }
  dec_hour_angle_text_position += std::to_string(dec_min) + "'";
  if (dec_sec < 10)
  {
    dec_hour_angle_text_position += "0";
  }
  std::stringstream ss_dec_sec;
  ss_dec_sec << std::fixed << std::setprecision(2) << dec_sec;
  dec_hour_angle_text_position += ss_dec_sec.str() + "\"";

  // Return value
  std::vector<std::string> hour_angle_text_stellar_position;
  hour_angle_text_stellar_position.push_back(ra_hour_angle_text_position);
  hour_angle_text_stellar_position.push_back(dec_hour_angle_text_position);
  return hour_angle_text_stellar_position;
}
