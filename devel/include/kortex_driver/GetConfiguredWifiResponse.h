// Generated by gencpp from file kortex_driver/GetConfiguredWifiResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETCONFIGUREDWIFIRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_GETCONFIGUREDWIFIRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/WifiConfiguration.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetConfiguredWifiResponse_
{
  typedef GetConfiguredWifiResponse_<ContainerAllocator> Type;

  GetConfiguredWifiResponse_()
    : output()  {
    }
  GetConfiguredWifiResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::WifiConfiguration_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetConfiguredWifiResponse_

typedef ::kortex_driver::GetConfiguredWifiResponse_<std::allocator<void> > GetConfiguredWifiResponse;

typedef boost::shared_ptr< ::kortex_driver::GetConfiguredWifiResponse > GetConfiguredWifiResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::GetConfiguredWifiResponse const> GetConfiguredWifiResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f527641f388c149b4396f8a32bb72e7";
  }

  static const char* value(const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4f527641f388c149ULL;
  static const uint64_t static_value2 = 0xb4396f8a32bb72e7ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetConfiguredWifiResponse";
  }

  static const char* value(const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "WifiConfiguration output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/WifiConfiguration\n"
"\n"
"Ssid ssid\n"
"string security_key\n"
"bool connect_automatically\n"
"================================================================================\n"
"MSG: kortex_driver/Ssid\n"
"\n"
"string identifier\n"
;
  }

  static const char* value(const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetConfiguredWifiResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetConfiguredWifiResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::WifiConfiguration_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETCONFIGUREDWIFIRESPONSE_H
