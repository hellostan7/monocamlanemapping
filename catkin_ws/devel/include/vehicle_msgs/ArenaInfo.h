// Generated by gencpp from file vehicle_msgs/ArenaInfo.msg
// DO NOT EDIT!


#ifndef VEHICLE_MSGS_MESSAGE_ARENAINFO_H
#define VEHICLE_MSGS_MESSAGE_ARENAINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <vehicle_msgs/LaneNet.h>
#include <vehicle_msgs/ObstacleSet.h>
#include <vehicle_msgs/VehicleSet.h>

namespace vehicle_msgs
{
template <class ContainerAllocator>
struct ArenaInfo_
{
  typedef ArenaInfo_<ContainerAllocator> Type;

  ArenaInfo_()
    : header()
    , lane_net()
    , obstacle_set()
    , vehicle_set()  {
    }
  ArenaInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , lane_net(_alloc)
    , obstacle_set(_alloc)
    , vehicle_set(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::vehicle_msgs::LaneNet_<ContainerAllocator>  _lane_net_type;
  _lane_net_type lane_net;

   typedef  ::vehicle_msgs::ObstacleSet_<ContainerAllocator>  _obstacle_set_type;
  _obstacle_set_type obstacle_set;

   typedef  ::vehicle_msgs::VehicleSet_<ContainerAllocator>  _vehicle_set_type;
  _vehicle_set_type vehicle_set;





  typedef boost::shared_ptr< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> const> ConstPtr;

}; // struct ArenaInfo_

typedef ::vehicle_msgs::ArenaInfo_<std::allocator<void> > ArenaInfo;

typedef boost::shared_ptr< ::vehicle_msgs::ArenaInfo > ArenaInfoPtr;
typedef boost::shared_ptr< ::vehicle_msgs::ArenaInfo const> ArenaInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_msgs::ArenaInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_msgs::ArenaInfo_<ContainerAllocator1> & lhs, const ::vehicle_msgs::ArenaInfo_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.lane_net == rhs.lane_net &&
    lhs.obstacle_set == rhs.obstacle_set &&
    lhs.vehicle_set == rhs.vehicle_set;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_msgs::ArenaInfo_<ContainerAllocator1> & lhs, const ::vehicle_msgs::ArenaInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9211a6cea105001098b4c5848c383814";
  }

  static const char* value(const ::vehicle_msgs::ArenaInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9211a6cea1050010ULL;
  static const uint64_t static_value2 = 0x98b4c5848c383814ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_msgs/ArenaInfo";
  }

  static const char* value(const ::vehicle_msgs::ArenaInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"LaneNet     lane_net\n"
"ObstacleSet obstacle_set\n"
"VehicleSet  vehicle_set\n"
"\n"
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
"MSG: vehicle_msgs/LaneNet\n"
"Header header\n"
"\n"
"Lane[] lanes\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/Lane\n"
"Header header\n"
"\n"
"int32 id\n"
"int32 dir\n"
"\n"
"int32[] child_id\n"
"int32[] father_id\n"
"\n"
"int32 l_lane_id\n"
"bool l_change_avbl\n"
"\n"
"int32 r_lane_id\n"
"bool r_change_avbl\n"
"\n"
"string behavior\n"
"\n"
"float32 length\n"
"\n"
"geometry_msgs/Point start_point\n"
"geometry_msgs/Point final_point\n"
"\n"
"geometry_msgs/Point[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/ObstacleSet\n"
"Header header\n"
"\n"
"CircleObstacle[] obs_circle\n"
"PolygonObstacle[] obs_polygon\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/CircleObstacle\n"
"Header header\n"
"\n"
"int32 id\n"
"Circle circle\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/Circle\n"
"geometry_msgs/Point center\n"
"float32 radius\n"
"\n"
"================================================================================\n"
"MSG: vehicle_msgs/PolygonObstacle\n"
"Header header\n"
"\n"
"int32 id\n"
"geometry_msgs/Polygon polygon\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: vehicle_msgs/VehicleSet\n"
"Header header\n"
"\n"
"Vehicle[] vehicles\n"
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
;
  }

  static const char* value(const ::vehicle_msgs::ArenaInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.lane_net);
      stream.next(m.obstacle_set);
      stream.next(m.vehicle_set);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArenaInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_msgs::ArenaInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_msgs::ArenaInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "lane_net: ";
    s << std::endl;
    Printer< ::vehicle_msgs::LaneNet_<ContainerAllocator> >::stream(s, indent + "  ", v.lane_net);
    s << indent << "obstacle_set: ";
    s << std::endl;
    Printer< ::vehicle_msgs::ObstacleSet_<ContainerAllocator> >::stream(s, indent + "  ", v.obstacle_set);
    s << indent << "vehicle_set: ";
    s << std::endl;
    Printer< ::vehicle_msgs::VehicleSet_<ContainerAllocator> >::stream(s, indent + "  ", v.vehicle_set);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_MSGS_MESSAGE_ARENAINFO_H
