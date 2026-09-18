#pragma once
#include <ros/msg.h>
namespace ros {
inline Msg* lastPublished = nullptr;
template<class M> class Subscriber {
 public:
  inline static void(*callback)(const M&) = nullptr;
  Subscriber(const char*, void(*cb)(const M&)) { callback=cb; }
};
class Publisher {
 public:
  Publisher(const char*, Msg*) {}
  void publish(Msg* message) { lastPublished=message; }
};
template<class Hardware,int Subs,int Pubs,int Input,int Output> class NodeHandle_ {
 public:
  void initNode() {}
  void spinOnce() {}
  template<class M> void subscribe(Subscriber<M>&) {}
  void advertise(Publisher&) {}
};
}
