// Generated by gencpp from file kortex_driver/MotorCommand.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_MOTORCOMMAND_H
#define KORTEX_DRIVER_MESSAGE_MOTORCOMMAND_H


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
struct MotorCommand_
{
  typedef MotorCommand_<ContainerAllocator> Type;

  MotorCommand_()
    : motor_id(0)
    , position(0.0)
    , velocity(0.0)
    , force(0.0)  {
    }
  MotorCommand_(const ContainerAllocator& _alloc)
    : motor_id(0)
    , position(0.0)
    , velocity(0.0)
    , force(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _motor_id_type;
  _motor_id_type motor_id;

   typedef float _position_type;
  _position_type position;

   typedef float _velocity_type;
  _velocity_type velocity;

   typedef float _force_type;
  _force_type force;





  typedef boost::shared_ptr< ::kortex_driver::MotorCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::MotorCommand_<ContainerAllocator> const> ConstPtr;

}; // struct MotorCommand_

typedef ::kortex_driver::MotorCommand_<std::allocator<void> > MotorCommand;

typedef boost::shared_ptr< ::kortex_driver::MotorCommand > MotorCommandPtr;
typedef boost::shared_ptr< ::kortex_driver::MotorCommand const> MotorCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::MotorCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::MotorCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::MotorCommand_<ContainerAllocator1> & lhs, const ::kortex_driver::MotorCommand_<ContainerAllocator2> & rhs)
{
  return lhs.motor_id == rhs.motor_id &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.force == rhs.force;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::MotorCommand_<ContainerAllocator1> & lhs, const ::kortex_driver::MotorCommand_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MotorCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::MotorCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MotorCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::MotorCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MotorCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::MotorCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "674b1a78bcad58fdb60d71acf839bd2f";
  }

  static const char* value(const ::kortex_driver::MotorCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x674b1a78bcad58fdULL;
  static const uint64_t static_value2 = 0xb60d71acf839bd2fULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/MotorCommand";
  }

  static const char* value(const ::kortex_driver::MotorCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::MotorCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"uint32 motor_id\n"
"float32 position\n"
"float32 velocity\n"
"float32 force\n"
;
  }

  static const char* value(const ::kortex_driver::MotorCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::MotorCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.motor_id);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.force);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotorCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::MotorCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::MotorCommand_<ContainerAllocator>& v)
  {
    s << indent << "motor_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.motor_id);
    s << indent << "position: ";
    Printer<float>::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
    s << indent << "force: ";
    Printer<float>::stream(s, indent + "  ", v.force);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_MOTORCOMMAND_H
