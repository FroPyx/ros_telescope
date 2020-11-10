#ifndef _ROS_telescope_msgs_MotorsStatusRaw_h
#define _ROS_telescope_msgs_MotorsStatusRaw_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace telescope_msgs
{

  class MotorsStatusRaw : public ros::Msg
  {
    public:
      typedef bool _motors_enabled_type;
      _motors_enabled_type motors_enabled;
      typedef int32_t _ra_motor_position_type;
      _ra_motor_position_type ra_motor_position;
      typedef int32_t _dec_motor_position_type;
      _dec_motor_position_type dec_motor_position;

    MotorsStatusRaw():
      motors_enabled(0),
      ra_motor_position(0),
      dec_motor_position(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motors_enabled;
      u_motors_enabled.real = this->motors_enabled;
      *(outbuffer + offset + 0) = (u_motors_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motors_enabled);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_motor_position;
      u_ra_motor_position.real = this->ra_motor_position;
      *(outbuffer + offset + 0) = (u_ra_motor_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_motor_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_motor_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_motor_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_motor_position);
      union {
        int32_t real;
        uint32_t base;
      } u_dec_motor_position;
      u_dec_motor_position.real = this->dec_motor_position;
      *(outbuffer + offset + 0) = (u_dec_motor_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dec_motor_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dec_motor_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dec_motor_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dec_motor_position);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motors_enabled;
      u_motors_enabled.base = 0;
      u_motors_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motors_enabled = u_motors_enabled.real;
      offset += sizeof(this->motors_enabled);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_motor_position;
      u_ra_motor_position.base = 0;
      u_ra_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_motor_position = u_ra_motor_position.real;
      offset += sizeof(this->ra_motor_position);
      union {
        int32_t real;
        uint32_t base;
      } u_dec_motor_position;
      u_dec_motor_position.base = 0;
      u_dec_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dec_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dec_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dec_motor_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dec_motor_position = u_dec_motor_position.real;
      offset += sizeof(this->dec_motor_position);
     return offset;
    }

    const char * getType(){ return "telescope_msgs/MotorsStatusRaw"; };
    const char * getMD5(){ return "810631f98374a5439a4b288f77c71895"; };

  };

}
#endif
