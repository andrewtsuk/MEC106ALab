// Generated by gencpp from file my_chatter/TimestampString.msg
// DO NOT EDIT!


#ifndef MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H
#define MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_chatter
{
template <class ContainerAllocator>
struct TimestampString_
{
  typedef TimestampString_<ContainerAllocator> Type;

  TimestampString_()
    : message()
    , time(0.0)  {
    }
  TimestampString_(const ContainerAllocator& _alloc)
    : message(_alloc)
    , time(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _message_type;
  _message_type message;

   typedef double _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::my_chatter::TimestampString_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_chatter::TimestampString_<ContainerAllocator> const> ConstPtr;

}; // struct TimestampString_

typedef ::my_chatter::TimestampString_<std::allocator<void> > TimestampString;

typedef boost::shared_ptr< ::my_chatter::TimestampString > TimestampStringPtr;
typedef boost::shared_ptr< ::my_chatter::TimestampString const> TimestampStringConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_chatter::TimestampString_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_chatter::TimestampString_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace my_chatter

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'my_chatter': ['/home/cc/ee106a/fl21/class/ee106a-acu/ros_workspaces/lab2/lab2/src/my_chatter/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::my_chatter::TimestampString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_chatter::TimestampString_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_chatter::TimestampString_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_chatter::TimestampString_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7ebcdee836ce6802abe87b1e6b9209eb";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7ebcdee836ce6802ULL;
  static const uint64_t static_value2 = 0xabe87b1e6b9209ebULL;
};

template<class ContainerAllocator>
struct DataType< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_chatter/TimestampString";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string message\n\
float64 time\n\
";
  }

  static const char* value(const ::my_chatter::TimestampString_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_chatter::TimestampString_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.message);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TimestampString_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_chatter::TimestampString_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_chatter::TimestampString_<ContainerAllocator>& v)
  {
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.message);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_CHATTER_MESSAGE_TIMESTAMPSTRING_H
