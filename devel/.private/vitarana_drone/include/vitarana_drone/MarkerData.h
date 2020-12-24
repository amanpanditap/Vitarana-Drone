// Generated by gencpp from file vitarana_drone/MarkerData.msg
// DO NOT EDIT!


#ifndef VITARANA_DRONE_MESSAGE_MARKERDATA_H
#define VITARANA_DRONE_MESSAGE_MARKERDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vitarana_drone
{
template <class ContainerAllocator>
struct MarkerData_
{
  typedef MarkerData_<ContainerAllocator> Type;

  MarkerData_()
    : marker_id(0)
    , err_x_m(0.0)
    , err_y_m(0.0)  {
    }
  MarkerData_(const ContainerAllocator& _alloc)
    : marker_id(0)
    , err_x_m(0.0)
    , err_y_m(0.0)  {
  (void)_alloc;
    }



   typedef int8_t _marker_id_type;
  _marker_id_type marker_id;

   typedef double _err_x_m_type;
  _err_x_m_type err_x_m;

   typedef double _err_y_m_type;
  _err_y_m_type err_y_m;





  typedef boost::shared_ptr< ::vitarana_drone::MarkerData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vitarana_drone::MarkerData_<ContainerAllocator> const> ConstPtr;

}; // struct MarkerData_

typedef ::vitarana_drone::MarkerData_<std::allocator<void> > MarkerData;

typedef boost::shared_ptr< ::vitarana_drone::MarkerData > MarkerDataPtr;
typedef boost::shared_ptr< ::vitarana_drone::MarkerData const> MarkerDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vitarana_drone::MarkerData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vitarana_drone::MarkerData_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vitarana_drone::MarkerData_<ContainerAllocator1> & lhs, const ::vitarana_drone::MarkerData_<ContainerAllocator2> & rhs)
{
  return lhs.marker_id == rhs.marker_id &&
    lhs.err_x_m == rhs.err_x_m &&
    lhs.err_y_m == rhs.err_y_m;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vitarana_drone::MarkerData_<ContainerAllocator1> & lhs, const ::vitarana_drone::MarkerData_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vitarana_drone

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::vitarana_drone::MarkerData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vitarana_drone::MarkerData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vitarana_drone::MarkerData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vitarana_drone::MarkerData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vitarana_drone::MarkerData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vitarana_drone::MarkerData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vitarana_drone::MarkerData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "07f88596d90f2c67e8f564a8e85f1ff2";
  }

  static const char* value(const ::vitarana_drone::MarkerData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x07f88596d90f2c67ULL;
  static const uint64_t static_value2 = 0xe8f564a8e85f1ff2ULL;
};

template<class ContainerAllocator>
struct DataType< ::vitarana_drone::MarkerData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vitarana_drone/MarkerData";
  }

  static const char* value(const ::vitarana_drone::MarkerData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vitarana_drone::MarkerData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 marker_id\n"
"float64 err_x_m\n"
"float64 err_y_m\n"
;
  }

  static const char* value(const ::vitarana_drone::MarkerData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vitarana_drone::MarkerData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.marker_id);
      stream.next(m.err_x_m);
      stream.next(m.err_y_m);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MarkerData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vitarana_drone::MarkerData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vitarana_drone::MarkerData_<ContainerAllocator>& v)
  {
    s << indent << "marker_id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.marker_id);
    s << indent << "err_x_m: ";
    Printer<double>::stream(s, indent + "  ", v.err_x_m);
    s << indent << "err_y_m: ";
    Printer<double>::stream(s, indent + "  ", v.err_y_m);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VITARANA_DRONE_MESSAGE_MARKERDATA_H
