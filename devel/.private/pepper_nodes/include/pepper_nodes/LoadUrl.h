// Generated by gencpp from file pepper_nodes/LoadUrl.msg
// DO NOT EDIT!


#ifndef PEPPER_NODES_MESSAGE_LOADURL_H
#define PEPPER_NODES_MESSAGE_LOADURL_H

#include <ros/service_traits.h>


#include <pepper_nodes/LoadUrlRequest.h>
#include <pepper_nodes/LoadUrlResponse.h>


namespace pepper_nodes
{

struct LoadUrl
{

typedef LoadUrlRequest Request;
typedef LoadUrlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct LoadUrl
} // namespace pepper_nodes


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pepper_nodes::LoadUrl > {
  static const char* value()
  {
    return "5562f0f326dc984bc777bae8e1589603";
  }

  static const char* value(const ::pepper_nodes::LoadUrl&) { return value(); }
};

template<>
struct DataType< ::pepper_nodes::LoadUrl > {
  static const char* value()
  {
    return "pepper_nodes/LoadUrl";
  }

  static const char* value(const ::pepper_nodes::LoadUrl&) { return value(); }
};


// service_traits::MD5Sum< ::pepper_nodes::LoadUrlRequest> should match
// service_traits::MD5Sum< ::pepper_nodes::LoadUrl >
template<>
struct MD5Sum< ::pepper_nodes::LoadUrlRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pepper_nodes::LoadUrl >::value();
  }
  static const char* value(const ::pepper_nodes::LoadUrlRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pepper_nodes::LoadUrlRequest> should match
// service_traits::DataType< ::pepper_nodes::LoadUrl >
template<>
struct DataType< ::pepper_nodes::LoadUrlRequest>
{
  static const char* value()
  {
    return DataType< ::pepper_nodes::LoadUrl >::value();
  }
  static const char* value(const ::pepper_nodes::LoadUrlRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pepper_nodes::LoadUrlResponse> should match
// service_traits::MD5Sum< ::pepper_nodes::LoadUrl >
template<>
struct MD5Sum< ::pepper_nodes::LoadUrlResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pepper_nodes::LoadUrl >::value();
  }
  static const char* value(const ::pepper_nodes::LoadUrlResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pepper_nodes::LoadUrlResponse> should match
// service_traits::DataType< ::pepper_nodes::LoadUrl >
template<>
struct DataType< ::pepper_nodes::LoadUrlResponse>
{
  static const char* value()
  {
    return DataType< ::pepper_nodes::LoadUrl >::value();
  }
  static const char* value(const ::pepper_nodes::LoadUrlResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PEPPER_NODES_MESSAGE_LOADURL_H
