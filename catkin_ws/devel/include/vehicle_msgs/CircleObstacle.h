// Generated by gencpp from file vehicle_msgs/CircleObstacle.msg
// DO NOT EDIT!


#ifndef VEHICLE_MSGS_MESSAGE_CIRCLEOBSTACLE_H
#define VEHICLE_MSGS_MESSAGE_CIRCLEOBSTACLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <vehicle_msgs/Circle.h>

namespace vehicle_msgs
{
template <class ContainerAllocator>
struct CircleObstacle_
{
  typedef CircleObstacle_<ContainerAllocator> Type;

  CircleObstacle_()
    : header()
    , id(0)
    , circle()  {
    }
  CircleObstacle_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , circle(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _id_type;
  _id_type id;

   typedef  ::vehicle_msgs::Circle_<ContainerAllocator>  _circle_type;
  _circle_type circle;





  typedef boost::shared_ptr< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> const> ConstPtr;

}; // struct CircleObstacle_

typedef ::vehicle_msgs::CircleObstacle_<std::allocator<void> > CircleObstacle;

typedef boost::shared_ptr< ::vehicle_msgs::CircleObstacle > CircleObstaclePtr;
typedef boost::shared_ptr< ::vehicle_msgs::CircleObstacle const> CircleObstacleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_msgs::CircleObstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_msgs::CircleObstacle_<ContainerAllocator1> & lhs, const ::vehicle_msgs::CircleObstacle_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.circle == rhs.circle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_msgs::CircleObstacle_<ContainerAllocator1> & lhs, const ::vehicle_msgs::CircleObstacle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "31dc893867f486c262045e463fa7c408";
  }

  static const char* value(const ::vehicle_msgs::CircleObstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x31dc893867f486c2ULL;
  static const uint64_t static_value2 = 0x62045e463fa7c408ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_msgs/CircleObstacle";
  }

  static const char* value(const ::vehicle_msgs::CircleObstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"int32 id\n"
"Circle circle\n"
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
"MSG: vehicle_msgs/Circle\n"
"geometry_msgs/Point center\n"
"float32 radius\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::vehicle_msgs::CircleObstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.circle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CircleObstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_msgs::CircleObstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_msgs::CircleObstacle_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "circle: ";
    s << std::endl;
    Printer< ::vehicle_msgs::Circle_<ContainerAllocator> >::stream(s, indent + "  ", v.circle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_MSGS_MESSAGE_CIRCLEOBSTACLE_H
