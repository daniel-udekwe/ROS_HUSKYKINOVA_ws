// Generated by gencpp from file kortex_driver/DesiredSpeeds.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_DESIREDSPEEDS_H
#define KORTEX_DRIVER_MESSAGE_DESIREDSPEEDS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kortex_driver
{
template <class ContainerAllocator>
struct DesiredSpeeds_
{
  typedef DesiredSpeeds_<ContainerAllocator> Type;

  DesiredSpeeds_()
    : linear(0.0)
    , angular(0.0)
    , joint_speed()  {
    }
  DesiredSpeeds_(const ContainerAllocator& _alloc)
    : linear(0.0)
    , angular(0.0)
    , joint_speed(_alloc)  {
  (void)_alloc;
    }



   typedef float _linear_type;
  _linear_type linear;

   typedef float _angular_type;
  _angular_type angular;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _joint_speed_type;
  _joint_speed_type joint_speed;





  typedef boost::shared_ptr< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> const> ConstPtr;

}; // struct DesiredSpeeds_

typedef ::kortex_driver::DesiredSpeeds_<std::allocator<void> > DesiredSpeeds;

typedef boost::shared_ptr< ::kortex_driver::DesiredSpeeds > DesiredSpeedsPtr;
typedef boost::shared_ptr< ::kortex_driver::DesiredSpeeds const> DesiredSpeedsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::DesiredSpeeds_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::DesiredSpeeds_<ContainerAllocator1> & lhs, const ::kortex_driver::DesiredSpeeds_<ContainerAllocator2> & rhs)
{
  return lhs.linear == rhs.linear &&
    lhs.angular == rhs.angular &&
    lhs.joint_speed == rhs.joint_speed;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::DesiredSpeeds_<ContainerAllocator1> & lhs, const ::kortex_driver::DesiredSpeeds_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9f61e0fe165a7f46b2508fd0832ff820";
  }

  static const char* value(const ::kortex_driver::DesiredSpeeds_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9f61e0fe165a7f46ULL;
  static const uint64_t static_value2 = 0xb2508fd0832ff820ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/DesiredSpeeds";
  }

  static const char* value(const ::kortex_driver::DesiredSpeeds_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"float32 linear\n"
"float32 angular\n"
"float32[] joint_speed\n"
;
  }

  static const char* value(const ::kortex_driver::DesiredSpeeds_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.linear);
      stream.next(m.angular);
      stream.next(m.joint_speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DesiredSpeeds_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::DesiredSpeeds_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::DesiredSpeeds_<ContainerAllocator>& v)
  {
    s << indent << "linear: ";
    Printer<float>::stream(s, indent + "  ", v.linear);
    s << indent << "angular: ";
    Printer<float>::stream(s, indent + "  ", v.angular);
    s << indent << "joint_speed[]" << std::endl;
    for (size_t i = 0; i < v.joint_speed.size(); ++i)
    {
      s << indent << "  joint_speed[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.joint_speed[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_DESIREDSPEEDS_H
