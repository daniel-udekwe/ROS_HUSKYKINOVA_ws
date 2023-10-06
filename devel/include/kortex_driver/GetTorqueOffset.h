// Generated by gencpp from file kortex_driver/GetTorqueOffset.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETTORQUEOFFSET_H
#define KORTEX_DRIVER_MESSAGE_GETTORQUEOFFSET_H

#include <ros/service_traits.h>


#include <kortex_driver/GetTorqueOffsetRequest.h>
#include <kortex_driver/GetTorqueOffsetResponse.h>


namespace kortex_driver
{

struct GetTorqueOffset
{

typedef GetTorqueOffsetRequest Request;
typedef GetTorqueOffsetResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetTorqueOffset
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::GetTorqueOffset > {
  static const char* value()
  {
    return "6ed1621e934f6480be9371fb25fd7ea1";
  }

  static const char* value(const ::kortex_driver::GetTorqueOffset&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::GetTorqueOffset > {
  static const char* value()
  {
    return "kortex_driver/GetTorqueOffset";
  }

  static const char* value(const ::kortex_driver::GetTorqueOffset&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::GetTorqueOffsetRequest> should match
// service_traits::MD5Sum< ::kortex_driver::GetTorqueOffset >
template<>
struct MD5Sum< ::kortex_driver::GetTorqueOffsetRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetTorqueOffset >::value();
  }
  static const char* value(const ::kortex_driver::GetTorqueOffsetRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetTorqueOffsetRequest> should match
// service_traits::DataType< ::kortex_driver::GetTorqueOffset >
template<>
struct DataType< ::kortex_driver::GetTorqueOffsetRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetTorqueOffset >::value();
  }
  static const char* value(const ::kortex_driver::GetTorqueOffsetRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::GetTorqueOffsetResponse> should match
// service_traits::MD5Sum< ::kortex_driver::GetTorqueOffset >
template<>
struct MD5Sum< ::kortex_driver::GetTorqueOffsetResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::GetTorqueOffset >::value();
  }
  static const char* value(const ::kortex_driver::GetTorqueOffsetResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::GetTorqueOffsetResponse> should match
// service_traits::DataType< ::kortex_driver::GetTorqueOffset >
template<>
struct DataType< ::kortex_driver::GetTorqueOffsetResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::GetTorqueOffset >::value();
  }
  static const char* value(const ::kortex_driver::GetTorqueOffsetResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETTORQUEOFFSET_H