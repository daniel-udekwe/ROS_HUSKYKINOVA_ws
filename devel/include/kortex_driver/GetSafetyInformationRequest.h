// Generated by gencpp from file kortex_driver/GetSafetyInformationRequest.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_GETSAFETYINFORMATIONREQUEST_H
#define KORTEX_DRIVER_MESSAGE_GETSAFETYINFORMATIONREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/SafetyHandle.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct GetSafetyInformationRequest_
{
  typedef GetSafetyInformationRequest_<ContainerAllocator> Type;

  GetSafetyInformationRequest_()
    : input()  {
    }
  GetSafetyInformationRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::SafetyHandle_<ContainerAllocator>  _input_type;
  _input_type input;





  typedef boost::shared_ptr< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetSafetyInformationRequest_

typedef ::kortex_driver::GetSafetyInformationRequest_<std::allocator<void> > GetSafetyInformationRequest;

typedef boost::shared_ptr< ::kortex_driver::GetSafetyInformationRequest > GetSafetyInformationRequestPtr;
typedef boost::shared_ptr< ::kortex_driver::GetSafetyInformationRequest const> GetSafetyInformationRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator2> & rhs)
{
  return lhs.input == rhs.input;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator1> & lhs, const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "28fc5544fa28c41b8590a221a4f4db30";
  }

  static const char* value(const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x28fc5544fa28c41bULL;
  static const uint64_t static_value2 = 0x8590a221a4f4db30ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/GetSafetyInformationRequest";
  }

  static const char* value(const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "SafetyHandle input\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/SafetyHandle\n"
"\n"
"uint32 identifier\n"
;
  }

  static const char* value(const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetSafetyInformationRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::GetSafetyInformationRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    s << std::endl;
    Printer< ::kortex_driver::SafetyHandle_<ContainerAllocator> >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_GETSAFETYINFORMATIONREQUEST_H