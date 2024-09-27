// Generated by gencpp from file vehicle_msgs/VehicleSet.msg
// DO NOT EDIT!


#ifndef VEHICLE_MSGS_MESSAGE_VEHICLESET_H
#define VEHICLE_MSGS_MESSAGE_VEHICLESET_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <vehicle_msgs/Vehicle.h>

namespace vehicle_msgs
{
template <class ContainerAllocator>
struct VehicleSet_
{
  typedef VehicleSet_<ContainerAllocator> Type;

  VehicleSet_()
    : header()
    , vehicles()  {
    }
  VehicleSet_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vehicles(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::vehicle_msgs::Vehicle_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::vehicle_msgs::Vehicle_<ContainerAllocator> >> _vehicles_type;
  _vehicles_type vehicles;





  typedef boost::shared_ptr< ::vehicle_msgs::VehicleSet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_msgs::VehicleSet_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleSet_

typedef ::vehicle_msgs::VehicleSet_<std::allocator<void> > VehicleSet;

typedef boost::shared_ptr< ::vehicle_msgs::VehicleSet > VehicleSetPtr;
typedef boost::shared_ptr< ::vehicle_msgs::VehicleSet const> VehicleSetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_msgs::VehicleSet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_msgs::VehicleSet_<ContainerAllocator1> & lhs, const ::vehicle_msgs::VehicleSet_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vehicles == rhs.vehicles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_msgs::VehicleSet_<ContainerAllocator1> & lhs, const ::vehicle_msgs::VehicleSet_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::VehicleSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::VehicleSet_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::VehicleSet_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1264823d8a48c6c06f03fb0f6c7a518";
  }

  static const char* value(const ::vehicle_msgs::VehicleSet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf1264823d8a48c6cULL;
  static const uint64_t static_value2 = 0x06f03fb0f6c7a518ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_msgs/VehicleSet";
  }

  static const char* value(const ::vehicle_msgs::VehicleSet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"Vehicle[] vehicles\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/Vehicle\n"
"Header header\n"
"std_msgs/Int32 id\n"
"std_msgs/String subclass\n"
"std_msgs/String type\n"
"VehicleParam param\n"
"State state\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Int32\n"
"int32 data\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/VehicleParam\n"
"# Kinematic\n"
"float32 width\n"
"float32 length\n"
"float32 wheel_base\n"
"float32 front_suspension\n"
"float32 rear_suspension\n"
"float32 max_steering_angle\n"
"\n"
"float32 d_cr # Length between rear axle to center of vehicle\n"
"\n"
"# Dynamic\n"
"float32 max_longitudinal_acc\n"
"float32 max_lateral_acc\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/State\n"
"Header header\n"
"geometry_msgs/Point vec_position\n"
"float64 angle\n"
"float64 curvature\n"
"float64 velocity\n"
"float64 acceleration\n"
"float64 steer\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::vehicle_msgs::VehicleSet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vehicles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleSet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_msgs::VehicleSet_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vehicles[]" << std::endl;
    for (size_t i = 0; i < v.vehicles.size(); ++i)
    {
      s << indent << "  vehicles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vehicle_msgs::Vehicle_<ContainerAllocator> >::stream(s, indent + "    ", v.vehicles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_MSGS_MESSAGE_VEHICLESET_H
