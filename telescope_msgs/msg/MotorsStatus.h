#ifndef _ROS_telescope_msgs_MotorsStatus_h
#define _ROS_telescope_msgs_MotorsStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace telescope_msgs
{

  class MotorsStatus : public ros::Msg
  {
    public:
      typedef bool _motors_connected_type;
      _motors_connected_type motors_connected;
      typedef bool _motors_enabled_type;
      _motors_enabled_type motors_enabled;
      typedef float _ra_motor_position_rad_type;
      _ra_motor_position_rad_type ra_motor_position_rad;
      typedef float _dec_motor_position_rad_type;
      _dec_motor_position_rad_type dec_motor_position_rad;
      typedef const char* _ra_motor_position_hms_type;
      _ra_motor_position_hms_type ra_motor_position_hms;
      typedef const char* _dec_motor_position_dms_type;
      _dec_motor_position_dms_type dec_motor_position_dms;

    MotorsStatus():
      motors_connected(0),
      motors_enabled(0),
      ra_motor_position_rad(0),
      dec_motor_position_rad(0),
      ra_motor_position_hms(""),
      dec_motor_position_dms("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motors_connected;
      u_motors_connected.real = this->motors_connected;
      *(outbuffer + offset + 0) = (u_motors_connected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motors_connected);
      union {
        bool real;
        uint8_t base;
      } u_motors_enabled;
      u_motors_enabled.real = this->motors_enabled;
      *(outbuffer + offset + 0) = (u_motors_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motors_enabled);
      offset += serializeAvrFloat64(outbuffer + offset, this->ra_motor_position_rad);
      offset += serializeAvrFloat64(outbuffer + offset, this->dec_motor_position_rad);
      uint32_t length_ra_motor_position_hms = strlen(this->ra_motor_position_hms);
      varToArr(outbuffer + offset, length_ra_motor_position_hms);
      offset += 4;
      memcpy(outbuffer + offset, this->ra_motor_position_hms, length_ra_motor_position_hms);
      offset += length_ra_motor_position_hms;
      uint32_t length_dec_motor_position_dms = strlen(this->dec_motor_position_dms);
      varToArr(outbuffer + offset, length_dec_motor_position_dms);
      offset += 4;
      memcpy(outbuffer + offset, this->dec_motor_position_dms, length_dec_motor_position_dms);
      offset += length_dec_motor_position_dms;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_motors_connected;
      u_motors_connected.base = 0;
      u_motors_connected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motors_connected = u_motors_connected.real;
      offset += sizeof(this->motors_connected);
      union {
        bool real;
        uint8_t base;
      } u_motors_enabled;
      u_motors_enabled.base = 0;
      u_motors_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motors_enabled = u_motors_enabled.real;
      offset += sizeof(this->motors_enabled);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ra_motor_position_rad));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dec_motor_position_rad));
      uint32_t length_ra_motor_position_hms;
      arrToVar(length_ra_motor_position_hms, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_ra_motor_position_hms; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_ra_motor_position_hms-1]=0;
      this->ra_motor_position_hms = (char *)(inbuffer + offset-1);
      offset += length_ra_motor_position_hms;
      uint32_t length_dec_motor_position_dms;
      arrToVar(length_dec_motor_position_dms, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_dec_motor_position_dms; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_dec_motor_position_dms-1]=0;
      this->dec_motor_position_dms = (char *)(inbuffer + offset-1);
      offset += length_dec_motor_position_dms;
     return offset;
    }

    const char * getType(){ return "telescope_msgs/MotorsStatus"; };
    const char * getMD5(){ return "d58bb483c5b7a70a9d5417c2c15fe59c"; };

  };

}
#endif
