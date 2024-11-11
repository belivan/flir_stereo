#ifndef OBJECT_DETECTION_MODULES_IMAGE_TRANSPORT_H
#define OBJECT_DETECTION_MODULES_IMAGE_TRANSPORT_H

#include <ros/ros.h>

namespace object_detection {

void disable_transports(ros::NodeHandle* nh, const std::string& name) {
  ROS_INFO_STREAM("Disabling plugins for " << name);
  nh->setParam(name + "/disable_pub_plugins",
               std::vector<std::string>{"image_transport/compressed",
                                        "image_transport/compressedDepth",
                                        "image_transport/theora"});
}

template <typename P, typename... Args>
void publish_if_subscribed(const P& pub, const Args&... args) {
  if (pub.getNumSubscribers() > 0) {
    pub.publish(args...);
  }
}
}  // namespace object_detection

#endif  // OBJECT_DETECTION_MODULES_IMAGE_TRANSPORT_H
