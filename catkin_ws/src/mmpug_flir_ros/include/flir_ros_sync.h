#ifndef FLIR_ROS_SYNC_H
#define FLIR_ROS_SYNC_H

#include "./fd_guard.h"
#include <linux/videodev2.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <atomic>
#include <thread>

#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <ctime>

#include <opencv2/opencv.hpp>

#include "../include/image_transport.h"
#include <cv_bridge/cv_bridge.h>

// #include <image_sharing/image_sharing.h>

namespace flir_ros_sync {

class FlirRos : public nodelet::Nodelet {
 public:
  FlirRos(){};
  virtual ~FlirRos();

 private:
  virtual void onInit() override;
  bool set_format(int fd, bool raw);
  bool request_buffers(int fd);
  bool start_streaming(int fd);
  void get_ros_param();
  void setup_ros();
  void setup_ros_names();
  void publish_frame(uint32_t bytes_used, ros::Time time);
  void publish_transforms(const ros::Time& time);
  void publish_transform(const ros::Time& time, const tf::Vector3& trans,
                         const tf::Quaternion& q, const std::string& from,
                         const std::string& to);
  sensor_msgs::ImagePtr rectify_image(
      const sensor_msgs::ImageConstPtr& image_msg,
      const sensor_msgs::CameraInfoConstPtr& camera_info_ptr);
  void get_frame_time(ros::Time& frame_time);

  object_detection::fd_guard fd;
  bool raw = true;
  int publish_image_sharing_every_n = 1000;
  int width = 640;   // use default if no param were given
  int height = 512;  // use default if no param were given
  int send_every_n=1;
  int count=0;
  void* buffer = nullptr;
  float timestampOffset = 0.0f; // signed value in seconds, true capture time = message receival time + offset, should be negative if message arrive later than capture

  // flir parameters
  int gain_mode = 2;
  int ffc_mode = 1;
  int sync_mode = 0;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // external trigger set
  int use_ext_sync = 1;

  // camera name and frame id
  std::string camera_name;
  std::string camera_topic_name;
  std::string rect_topic_name;
  std::string base_frame_id;
  std::string img_opt_frame_id;

  // image transport interfaces
  std::unique_ptr<image_transport::ImageTransport> it;
  image_transport::CameraPublisher image_pub;
  image_transport::Publisher rect_image_pub;
  //nv2ros::Publisher* nv_image_pub;
  std::string intrinsic_url;

  // camera intrinsics info
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo;

  // image geometry interface for rectification
  image_geometry::PinholeCameraModel cam_model;

  // tf broadcaster for camera_link and optical_frame
  tf2_ros::TransformBroadcaster br;

  std::atomic_bool stream{false};
  std::thread stream_thread;
};

}  // namespace flir_ros_sync

#endif  // FLIR_ROS_SYNC_H
