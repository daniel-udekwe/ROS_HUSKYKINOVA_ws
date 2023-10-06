// Generated by gencpp from file kortex_driver/SetControllerConfigurationRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONREQUEST_H
#define KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/ControllerConfiguration.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct SetControllerConfigurationRequest_
{
  typedef SetControllerConfigurationRequest_<ContainerAllocator> Type;

  SetControllerConfigurationRequest_()
    : input()  {
    }
  SetControllerConfigurationRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::ControllerConfiguration_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetControllerConfigurationRequest_

typedef ::kortex_driver::SetControllerConfigurationRequest_<std::allocator<void> > SetControllerConfigurationRequest;

typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationRequest > SetControllerConfigurationRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::SetControllerConfigurationRequest const> SetControllerConfigurationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "af14a3e0157001d2cf8a6624118d8c8d";
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaf14a3e0157001d2ULL;
  static const uint64_t static_value2 = 0xcf8a6624118d8c8dULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/SetControllerConfigurationRequest";
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ControllerConfiguration input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerConfiguration\n"
"\n"
"ControllerHandle handle\n"
"string name\n"
"MappingHandle active_mapping_handle\n"
"string analog_input_identifier_enum\n"
"string digital_input_identifier_enum\n"
"================================================================================\n"
"MSG: kortex_driver/ControllerHandle\n"
"\n"
"uint32 type\n"
"uint32 controller_identifier\n"
"================================================================================\n"
"MSG: kortex_driver/MappingHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetControllerConfigurationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::SetControllerConfigurationRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::ControllerConfiguration_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_SETCONTROLLERCONFIGURATIONREQUEST_H