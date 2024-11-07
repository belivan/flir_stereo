#ifndef FLIR_ROS_SYNC_H
#define FLIR_ROS_SYNC_H

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <atomic>
#include <thread>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ctime>

#include "./fd_guard.h"
#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>

#include "../include/image_transport.h"
#include <cv_bridge/cv_bridge.h>

// #include <image_sharing/image_sharing.h>

namespace flir_ros_sync {

struct CameraConfig {
    std::string camera_name;
    std::string intrinsic_url;
    int width = 640;
    int height = 512;
    int use_ext_sync = 1;
    int send_every_n = 1;
    bool raw = true;
    int gain_mode = 2;
    int ffc_mode = 1;
    float timestamp_offset = 0.0f;
};

struct DeviceInfo {
    int fd;
    void* buffer;
    std::string device_path;
    std::string serial_port;
};

struct PublisherContext {
    std::shared_ptr<image_transport::ImageTransport> it;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
    image_transport::CameraPublisher image_pub;
    image_transport::Publisher rect_image_pub;
    std::string camera_topic_name;
    std::string rect_topic_name;
    std::string base_frame_id;
    std::string img_opt_frame_id;
};

// Result type for error handling
template<typename T>
struct Result {
    bool success;
    std::string error_message;
    T value;
    
    static Result<T> ok(T val) {
        return Result<T>{true, "", std::move(val)};
    }
    
    static Result<T> error(std::string msg) {
        return Result<T>{false, std::move(msg), T{}};
    }
};

class FlirRos : public rclcpp::Node, public std::enable_shared_from_this<FlirRos> {
public:
    explicit FlirRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FlirRos();

    Result<void> initialize();

private:
    Result<void> initializeDevice();
    Result<void> setupROS();
    void streamingLoop();
    // ... keep other private method declarations ...

    CameraConfig config_;
    DeviceInfo device_;
    PublisherContext publishers_;
    
    object_detection::fd_guard fd;
    std::atomic_bool stream{false};
    std::thread stream_thread;
    int count = 0;

    tf2_ros::TransformBroadcaster br;
    image_geometry::PinholeCameraModel cam_model;
};

}  // namespace flir_ros_sync

#endif  // FLIR_ROS_SYNC_H
