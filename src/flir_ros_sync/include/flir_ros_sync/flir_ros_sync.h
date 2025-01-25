#ifndef FLIR_ROS_SYNC_H
#define FLIR_ROS_SYNC_H

// Standard includes
#include <memory>
#include <atomic>
#include <thread>
#include <string>
#include <ctime>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Project includes
#include "fd_guard.h"
#include "image_transport.h"

extern "C" {
    #include "EnumTypes.h"
    #include "ReturnCodes.h"
    #include "Client_API.h"
    #include "UART_Connector.h"
    #include "serialPortAdapter.h"
}

namespace flir_ros_sync {

// Color codes
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

struct CameraConfig {
    std::string camera_name = "flir";
    std::string intrinsic_url = "package://flir_ros_sync/data/camera_info/flir_intrinsics.yaml";
    int width = 640;
    int height = 512;
    int total_height = 514;
    bool raw = true;
    int gain_mode = 2;
    int ffc_mode = 1;
    int use_ext_sync = 1;
    int send_every_n = 1;
    double timestamp_offset = 0.0;
    int frame_rate = 10;
    int ffc_interval_mins = 3; // default 3 minutes

};

struct DeviceInfo {
    int fd = -1;
    void* buffer = nullptr;
    std::string device_path;
    std::string serial_port;
};

struct PublisherContext {
    // temp timestamp pub
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr timestamp_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ffc_status_pub_;

    std::shared_ptr<image_transport::ImageTransport> it;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo;
    image_transport::CameraPublisher image_pub;
    image_transport::Publisher rect_image_pub;
    std::string camera_topic_name;
    std::string rect_topic_name;
    std::string base_frame_id;
    std::string img_opt_frame_id;
};

class FlirRos : public rclcpp::Node{
public:
    explicit FlirRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~FlirRos();

    void initialize();

private:
    // Timestamp variables
    uint64_t timestamp_init_{0};  // Initial telemetry timestamp
    uint64_t system_time_init_{0}; // Initial system time
    bool first_frame_{true};

    // FFC variables
    bool last_ffc_status_{false};
    int last_ffc_frame_count_{1}; // Init to 1 because we do FFC from the start

    // CV Mat maps
    cv::Mat map1_, map2_; // Rectification maps
    bool maps_initialized_{false};

    // Initialization methods
    void loadParameters();
    void initializeDevice();
    void setupROS();
    void initializeTelemetry();

    // Streaming methods
    void streamingLoop();
    void extractTelemetryTimestamp(void* buffer, size_t buffer_size, rclcpp::Time& frame_time);
    void publishFrame(uint32_t bytes_used, const rclcpp::Time& time);
    sensor_msgs::msg::Image::SharedPtr rectify_image(
                            const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info);
    void publishTransforms(const rclcpp::Time& time);
    void publishTransform(const rclcpp::Time& time, const geometry_msgs::msg::Vector3& trans,
                          const tf2::Quaternion& q, const std::string& from,
                          const std::string& to);
    // void getFrameTime(rclcpp::Time& frame_time);

    // Utility methods
    bool setFormat(int fd, bool raw);
    bool requestBuffers(int fd);
    bool startStreaming(int fd);
    void checkNUCTableStatus();
    void performFFC();
    void getFFCFrameCount(uint32_t& ffc_frame_count);
    void getFFCStatus(int16_t& status);
    void publishFFCStatus(bool status);

    // Camera configuration and device info
    CameraConfig config_;
    DeviceInfo device_;
    PublisherContext publisher_;

    // ROS tools
    tf2_ros::TransformBroadcaster transform_broadcaster_;
    image_geometry::PinholeCameraModel cam_model_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Streaming control
    std::atomic_bool stream_active_{false};
    std::thread stream_thread_;
    int frame_count_ = 0;

    // Logging macros
    #define LOG_INFO(...) RCLCPP_INFO(this->get_logger(), ANSI_COLOR_GREEN "[INFO] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_ERROR(...) RCLCPP_ERROR(this->get_logger(), ANSI_COLOR_RED "[ERROR] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_FATAL(...) RCLCPP_FATAL(this->get_logger(), ANSI_COLOR_RED "[FATAL] " ANSI_COLOR_RESET __VA_ARGS__)
    #define LOG_WARN(...) RCLCPP_WARN(this->get_logger(), ANSI_COLOR_YELLOW "[WARN] " ANSI_COLOR_RESET __VA_ARGS__)
};

}  // namespace flir_ros_sync

#endif  // FLIR_ROS_SYNC_H
