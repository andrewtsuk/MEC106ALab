// Generated by gencpp from file turtle_patrol/PatrolRequest.msg
// DO NOT EDIT!


#ifndef TURTLE_PATROL_MESSAGE_PATROLREQUEST_H
#define TURTLE_PATROL_MESSAGE_PATROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace turtle_patrol
{
template <class ContainerAllocator>
struct PatrolRequest_
{
  typedef PatrolRequest_<ContainerAllocator> Type;

  PatrolRequest_()
    : vel(0.0)
    , omega(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  PatrolRequest_(const ContainerAllocator& _alloc)
    : vel(0.0)
    , omega(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef float _vel_type;
  _vel_type vel;

   typedef float _omega_type;
  _omega_type omega;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::turtle_patrol::PatrolRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turtle_patrol::PatrolRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PatrolRequest_

typedef ::turtle_patrol::PatrolRequest_<std::allocator<void> > PatrolRequest;

typedef boost::shared_ptr< ::turtle_patrol::PatrolRequest > PatrolRequestPtr;
typedef boost::shared_ptr< ::turtle_patrol::PatrolRequest const> PatrolRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::turtle_patrol::PatrolRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace turtle_patrol

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::turtle_patrol::PatrolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::turtle_patrol::PatrolRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turtle_patrol::PatrolRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "901dd5e4fb3ebf17f9414f84df1e0f05";
  }

  static const char* value(const ::turtle_patrol::PatrolRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x901dd5e4fb3ebf17ULL;
  static const uint64_t static_value2 = 0xf9414f84df1e0f05ULL;
};

template<class ContainerAllocator>
struct DataType< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "turtle_patrol/PatrolRequest";
  }

  static const char* value(const ::turtle_patrol::PatrolRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
float32 vel\n\
float32 omega\n\
float32 x\n\
float32 y\n\
float32 theta\n\
";
  }

  static const char* value(const ::turtle_patrol::PatrolRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vel);
      stream.next(m.omega);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PatrolRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::turtle_patrol::PatrolRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::turtle_patrol::PatrolRequest_<ContainerAllocator>& v)
  {
    s << indent << "vel: ";
    Printer<float>::stream(s, indent + "  ", v.vel);
    s << indent << "omega: ";
    Printer<float>::stream(s, indent + "  ", v.omega);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TURTLE_PATROL_MESSAGE_PATROLREQUEST_H
