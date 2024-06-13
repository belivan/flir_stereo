#ifndef OBJECT_DETECTION_MODULES_TF2_H
#define OBJECT_DETECTION_MODULES_TF2_H

#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace object_detection {
Eigen::Affine3f eigen_tf_from_tf2(const tf2::Transform& tf) {
  const auto& origin = tf.getOrigin();
  const auto& rotation = tf.getRotation();

  return Eigen::Affine3f(Eigen::Translation3f(origin[0], origin[1], origin[2]) *
                         Eigen::Quaternionf(rotation.w(), rotation.x(),
                                            rotation.y(), rotation.z()));
}

Eigen::Affine3f eigen_tf_from_pose(const geometry_msgs::Pose& pose) {
  const auto& trans = pose.position;
  const auto& rot = pose.orientation;

  return Eigen::Affine3f(Eigen::Translation3f(trans.x, trans.y, trans.z) *
                         Eigen::Quaternionf(rot.w, rot.x, rot.y, rot.z));
}

// Some of the frames people use have a leading "/" which is illegal. This
// function corrects for that.
inline std::string sanitize_frame(const std::string& input) {
  if (input.length() > 0 && input[0] == '/') {
    return input.substr(1);
  }

  return input;
}

}  // namespace object_detection

#endif  // OBJECT_DETECTION_MODULES_TF2_H
