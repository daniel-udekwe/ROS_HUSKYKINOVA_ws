// Generated by gencpp from file kortex_driver/ReadAllMappingsResponse.msg
// DO NOT EDIT!


#ifndef KORTEX_DRIVER_MESSAGE_READALLMAPPINGSRESPONSE_H
#define KORTEX_DRIVER_MESSAGE_READALLMAPPINGSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <kortex_driver/MappingList.h>

namespace kortex_driver
{
template <class ContainerAllocator>
struct ReadAllMappingsResponse_
{
  typedef ReadAllMappingsResponse_<ContainerAllocator> Type;

  ReadAllMappingsResponse_()
    : output()  {
    }
  ReadAllMappingsResponse_(const ContainerAllocator& _alloc)
    : output(_alloc)  {
  (void)_alloc;
    }



   typedef  ::kortex_driver::MappingList_<ContainerAllocator>  _output_type;
  _output_type output;





  typedef boost::shared_ptr< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ReadAllMappingsResponse_

typedef ::kortex_driver::ReadAllMappingsResponse_<std::allocator<void> > ReadAllMappingsResponse;

typedef boost::shared_ptr< ::kortex_driver::ReadAllMappingsResponse > ReadAllMappingsResponsePtr;
typedef boost::shared_ptr< ::kortex_driver::ReadAllMappingsResponse const> ReadAllMappingsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.output == rhs.output;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator1> & lhs, const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace kortex_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f318987fb4be88e1c786ff4d68235a14";
  }

  static const char* value(const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf318987fb4be88e1ULL;
  static const uint64_t static_value2 = 0xc786ff4d68235a14ULL;
};

template<class ContainerAllocator>
struct DataType< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kortex_driver/ReadAllMappingsResponse";
  }

  static const char* value(const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "MappingList output\n"
"\n"
"================================================================================\n"
"MSG: kortex_driver/MappingList\n"
"\n"
"Mapping[] mappings\n"
"================================================================================\n"
"MSG: kortex_driver/Mapping\n"
"\n"
"MappingHandle handle\n"
"string name\n"
"uint32 controller_identifier\n"
"MapGroupHandle active_map_group_handle\n"
"MapGroupHandle[] map_group_handles\n"
"MapHandle active_map_handle\n"
"MapHandle[] map_handles\n"
"string application_data\n"
"================================================================================\n"
"MSG: kortex_driver/MappingHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/MapGroupHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
"================================================================================\n"
"MSG: kortex_driver/MapHandle\n"
"\n"
"uint32 identifier\n"
"uint32 permission\n"
;
  }

  static const char* value(const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.output);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReadAllMappingsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kortex_driver::ReadAllMappingsResponse_<ContainerAllocator>& v)
  {
    s << indent << "output: ";
    s << std::endl;
    Printer< ::kortex_driver::MappingList_<ContainerAllocator> >::stream(s, indent + "  ", v.output);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KORTEX_DRIVER_MESSAGE_READALLMAPPINGSRESPONSE_H
