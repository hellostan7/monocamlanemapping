// Generated by gencpp from file vehicle_msgs/State.msg
// DO NOT EDIT!


#ifndef VEHICLE_MSGS_MESSAGE_STATE_H
#define VEHICLE_MSGS_MESSAGE_STATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace vehicle_msgs
{
template <class ContainerAllocator>
struct State_
{
  typedef State_<ContainerAllocator> Type;

  State_()
    : header()
    , vec_position()
    , angle(0.0)
    , curvature(0.0)
    , velocity(0.0)
    , acceleration(0.0)
    , steer(0.0)  {
    }
  State_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vec_position(_alloc)
    , angle(0.0)
    , curvature(0.0)
    , velocity(0.0)
    , acceleration(0.0)
    , steer(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _vec_position_type;
  _vec_position_type vec_position;

   typedef double _angle_type;
  _angle_type angle;

   typedef double _curvature_type;
  _curvature_type curvature;

   typedef double _velocity_type;
  _velocity_type velocity;

   typedef double _acceleration_type;
  _acceleration_type acceleration;

   typedef double _steer_type;
  _steer_type steer;





  typedef boost::shared_ptr< ::vehicle_msgs::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_msgs::State_<ContainerAllocator> const> ConstPtr;

}; // struct State_

typedef ::vehicle_msgs::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::vehicle_msgs::State > StatePtr;
typedef boost::shared_ptr< ::vehicle_msgs::State const> StateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_msgs::State_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_msgs::State_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_msgs::State_<ContainerAllocator1> & lhs, const ::vehicle_msgs::State_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.vec_position == rhs.vec_position &&
    lhs.angle == rhs.angle &&
    lhs.curvature == rhs.curvature &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration &&
    lhs.steer == rhs.steer;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_msgs::State_<ContainerAllocator1> & lhs, const ::vehicle_msgs::State_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::State_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::State_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::State_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_msgs::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5996f819702fbdc617ef5bcadf1785ea";
  }

  static const char* value(const ::vehicle_msgs::State_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5996f819702fbdc6ULL;
  static const uint64_t static_value2 = 0x17ef5bcadf1785eaULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_msgs::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_msgs/State";
  }

  static const char* value(const ::vehicle_msgs::State_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_msgs::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Point vec_position\n"
"float64 angle\n"
"float64 curvature\n"
"float64 velocity\n"
"float64 acceleration\n"
"float64 steer\n"
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
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::vehicle_msgs::State_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_msgs::State_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vec_position);
      stream.next(m.angle);
      stream.next(m.curvature);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.steer);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct State_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_msgs::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_msgs::State_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vec_position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.vec_position);
    s << indent << "angle: ";
    Printer<double>::stream(s, indent + "  ", v.angle);
    s << indent << "curvature: ";
    Printer<double>::stream(s, indent + "  ", v.curvature);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    Printer<double>::stream(s, indent + "  ", v.acceleration);
    s << indent << "steer: ";
    Printer<double>::stream(s, indent + "  ", v.steer);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_MSGS_MESSAGE_STATE_H
