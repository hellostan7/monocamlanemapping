// Generated by gencpp from file vehicle_msgs/Circle.msg
// DO NOT EDIT!


#ifndef VEHICLE_MSGS_MESSAGE_CIRCLE_H
#define VEHICLE_MSGS_MESSAGE_CIRCLE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace vehicle_msgs
{
template <class ContainerAllocator>
struct Circle_
{
  typedef Circle_<ContainerAllocator> Type;

  Circle_()
    : center()
    , radius(0.0)  {
    }
  Circle_(const ContainerAllocator& _alloc)
    : center(_alloc)
    , radius(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _center_type;
  _center_type center;

   typedef float _radius_type;
  _radius_type radius;





  typedef boost::shared_ptr< ::vehicle_msgs::Circle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vehicle_msgs::Circle_<ContainerAllocator> const> ConstPtr;

}; // struct Circle_

typedef ::vehicle_msgs::Circle_<std::allocator<void> > Circle;

typedef boost::shared_ptr< ::vehicle_msgs::Circle > CirclePtr;
typedef boost::shared_ptr< ::vehicle_msgs::Circle const> CircleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vehicle_msgs::Circle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vehicle_msgs::Circle_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vehicle_msgs::Circle_<ContainerAllocator1> & lhs, const ::vehicle_msgs::Circle_<ContainerAllocator2> & rhs)
{
  return lhs.center == rhs.center &&
    lhs.radius == rhs.radius;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vehicle_msgs::Circle_<ContainerAllocator1> & lhs, const ::vehicle_msgs::Circle_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vehicle_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::Circle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vehicle_msgs::Circle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::Circle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vehicle_msgs::Circle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::Circle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vehicle_msgs::Circle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vehicle_msgs::Circle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0346e2adfa9eef61749f32871bb34611";
  }

  static const char* value(const ::vehicle_msgs::Circle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0346e2adfa9eef61ULL;
  static const uint64_t static_value2 = 0x749f32871bb34611ULL;
};

template<class ContainerAllocator>
struct DataType< ::vehicle_msgs::Circle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vehicle_msgs/Circle";
  }

  static const char* value(const ::vehicle_msgs::Circle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vehicle_msgs::Circle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point center\n"
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

  static const char* value(const ::vehicle_msgs::Circle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vehicle_msgs::Circle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.center);
      stream.next(m.radius);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Circle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vehicle_msgs::Circle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vehicle_msgs::Circle_<ContainerAllocator>& v)
  {
    s << indent << "center: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.center);
    s << indent << "radius: ";
    Printer<float>::stream(s, indent + "  ", v.radius);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VEHICLE_MSGS_MESSAGE_CIRCLE_H
