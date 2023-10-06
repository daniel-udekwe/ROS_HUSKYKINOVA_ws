// Generated by gencpp from file kortex_driver/SendSelectedJointSpeedCommand.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SENDSELECTEDJOINTSPEEDCOMMAND_H
#define KORTEX_DRIVER_MESSAGE_SENDSELECTEDJOINTSPEEDCOMMAND_H

#include <ros/service_traits.h>


#include <kortex_driver/SendSelectedJointSpeedCommandRequest.h>
#include <kortex_driver/SendSelectedJointSpeedCommandResponse.h>


namespace kortex_driver
{

struct SendSelectedJointSpeedCommand
{

typedef SendSelectedJointSpeedCommandRequest Request;
typedef SendSelectedJointSpeedCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SendSelectedJointSpeedCommand
} // namespace kortex_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommand > {
  static const char* value()
  {
    return "966eae21a761830257b0f20dfcd24c8e";
  }

  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommand&) { return value(); }
};

template<>
struct DataType< ::kortex_driver::SendSelectedJointSpeedCommand > {
  static const char* value()
  {
    return "kortex_driver/SendSelectedJointSpeedCommand";
  }

  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommand&) { return value(); }
};


// service_traits::MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommandRequest> should match
// service_traits::MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommand >
template<>
struct MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SendSelectedJointSpeedCommandRequest> should match
// service_traits::DataType< ::kortex_driver::SendSelectedJointSpeedCommand >
template<>
struct DataType< ::kortex_driver::SendSelectedJointSpeedCommandRequest>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SendSelectedJointSpeedCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommandResponse> should match
// service_traits::MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommand >
template<>
struct MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::kortex_driver::SendSelectedJointSpeedCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::kortex_driver::SendSelectedJointSpeedCommandResponse> should match
// service_traits::DataType< ::kortex_driver::SendSelectedJointSpeedCommand >
template<>
struct DataType< ::kortex_driver::SendSelectedJointSpeedCommandResponse>
{
  static const char* value()
  {
    return DataType< ::kortex_driver::SendSelectedJointSpeedCommand >::value();
  }
  static const char* value(const ::kortex_driver::SendSelectedJointSpeedCommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SENDSELECTEDJOINTSPEEDCOMMAND_H
