#ifndef _ROS_SERVICE_Wheel_h
#define _ROS_SERVICE_Wheel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace my_service
{

static const char WHEEL[] = "my_service/Wheel";

  class WheelRequest : public ros::Msg
  {
    public:
      typedef float _target_linear_velocity_type;
      _target_linear_velocity_type target_linear_velocity;
      typedef float _target_angular_velocity_type;
      _target_angular_velocity_type target_angular_velocity;

    WheelRequest():
      target_linear_velocity(0),
      target_angular_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->target_linear_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->target_angular_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_linear_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->target_angular_velocity));
     return offset;
    }

    const char * getType(){ return WHEEL; };
    const char * getMD5(){ return "f0250a22f26b2d4fb70fdb58e85f8b9a"; };

  };

  class WheelResponse : public ros::Msg
  {
    public:
      typedef float _current_linear_velocity_type;
      _current_linear_velocity_type current_linear_velocity;
      typedef float _current_angular_velocity_type;
      _current_angular_velocity_type current_angular_velocity;

    WheelResponse():
      current_linear_velocity(0),
      current_angular_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->current_linear_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->current_angular_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_linear_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->current_angular_velocity));
     return offset;
    }

    const char * getType(){ return WHEEL; };
    const char * getMD5(){ return "056d03b83b9853a858cfe23bcf52551d"; };

  };

  class Wheel {
    public:
    typedef WheelRequest Request;
    typedef WheelResponse Response;
  };

}
#endif
