// Include librairies
#include <ros/ros.h>
#include <stellar_position_conversion/StellarPositionConversion.h>


int main(int argc, char** argv)
{

  // Test rad to degree
  std::cout << std::endl;
  std::cout << "Test rad to degree : " << std::endl;
  double rad = -0.154;
  double degree = radToDegree(rad);
  std::cout << rad << " rad = " << degree << " degree" << std::endl;

  // Test hour andle to rad
  std::cout << std::endl;
  std::cout << "Test hour angle to rad : " << std::endl;
  double hour_angle = -0.154;
  rad = hourAngleToRad(hour_angle);
  std::cout << hour_angle << " hour angle = " << rad << " rad" << std::endl;

  // Test hour andle to degree
  std::cout << std::endl;
  std::cout << "Test hour angle to degree : " << std::endl;
  hour_angle = -0.154;
  degree = hourAngleToDegree(hour_angle);
  std::cout << hour_angle << " hour angle = " << degree << " degree" << std::endl;

  // Test degree to rad
  std::cout << std::endl;
  std::cout << "Test degree to rad : " << std::endl;
  degree = -8.512;
  rad = degreeToRad(degree);
  std::cout << degree << " degree = " << rad << " rad" << std::endl;

  // Test rad to hour angle
  std::cout << std::endl;
  std::cout << "Test rad to hour angle : " << std::endl;
  rad = -1.512;
  hour_angle = radToHourAngle(rad);
  std::cout << rad << " rad = " << hour_angle << " hour angle" << std::endl;

  // Test degree to hour angle
  std::cout << std::endl;
  std::cout << "Test degree to hour angle : " << std::endl;
  degree = -1.512;
  hour_angle = degreeToHourAngle(degree);
  std::cout << degree << " degree = " << hour_angle << " hour angle" << std::endl;

  // Test degree to DMS
  std::cout << std::endl;
  std::cout << "Test degree to DMS : " << std::endl;
  degree = 0.0;
  std::string dms = degreeToDMS(degree);
  std::cout << degree << " degree = " << dms << " DMS" << std::endl;

  // Test rad to DMS
  std::cout << std::endl;
  std::cout << "Test rad to DMS : " << std::endl;
  rad = 2.512;
  dms = radToDMS(rad);
  std::cout << rad << " rad = " << dms << " DMS" << std::endl;

  // Test hour angle to DMS
  std::cout << std::endl;
  std::cout << "Test hour angle to DMS : " << std::endl;
  hour_angle = 2.512;
  dms = hourAngleToDMS(hour_angle);
  std::cout << hour_angle << " hour angle = " << dms << " DMS" << std::endl;

  // Test hour angle to HMS
  std::cout << std::endl;
  std::cout << "Test hour angle to HMS : " << std::endl;
  hour_angle = 2.512;
  std::string hms = hourAngleToHMS(hour_angle);
  std::cout << hour_angle << " hour angle = " << hms << " HMS" << std::endl;

  // Test degree to HMS
  std::cout << std::endl;
  std::cout << "Test degree to HMS : " << std::endl;
  degree = -232.64;
  hms = degreeToHMS(degree);
  std::cout << degree << " degree = " << hms << " HMS" << std::endl;

  // Test rad to HMS
  std::cout << std::endl;
  std::cout << "Test degree to HMS : " << std::endl;
  rad = -2.64;
  hms = radToHMS(rad);
  std::cout << rad << " rad = " << hms << " HMS" << std::endl;

  return 0;
}
