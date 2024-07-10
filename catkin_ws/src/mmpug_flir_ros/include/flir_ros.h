#ifndef FLIR_ROS_H
#define FLIR_ROS_H

#include <common/fd_guard.h>
#include <common/timer.h>
#include <linux/videodev2.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <atomic>
#include <thread>

#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

namespace flir_ros {

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

  std::string device_name;
  object_detection::fd_guard fd;
  bool raw = true;
  int width = 640;   // use default if no param were given
  int height = 512;  // use default if no param were given
  void* buffer = nullptr;

  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

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
  std::string intrinsic_url;

  // camera intrinsics info
  std::unique_ptr<camera_info_manager::CameraInfoManager> cinfo;

  // image geometry interface for rectification
  image_geometry::PinholeCameraModel cam_model;

  // tf broadcaster for camera_link and optical_frame
  tf2_ros::TransformBroadcaster br;

  std::atomic_bool stream{false};
  std::thread stream_thread;

  // This will only send 1 of N frames to get read. Count starts at 0 and allows
  // N-1 frames to get skipped before sending the first one to allow for any
  // sort of initialization the camera needs to do.
  int send_every_n = 1;
  int count = 0;

  ros::Time thread_start;

  object_detection::ThreadsafeTimer timer;
};

}  // namespace flir_ros

#endif  // FLIR_ROS_H
