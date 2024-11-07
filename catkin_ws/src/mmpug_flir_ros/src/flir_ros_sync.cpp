#include "../include/flir_ros_sync.h"

// Standard includes
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstdlib>
#include <chrono>

// V4L2 includes
#include <linux/videodev2.h>

// ROS includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace flir_ros_sync {

FlirRos::FlirRos(const rclcpp::NodeOptions& options)
    : Node("flir_ros_sync", options) {
    LOG_INFO("Initializing FLIR ROS2 Node");
}

FlirRos::~FlirRos() {
    LOG_INFO("Shutting down FLIR ROS2 Node");

    stream_active_.store(false);
    if (stream_thread_.joinable()) {
        stream_thread_.join();
    }

    if (device_.fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(device_.fd, VIDIOC_STREAMOFF, &type) < 0) {
            LOG_ERROR("Failed to stop streaming");
        }
    }
}

Result<void> FlirRos::initialize() {
    // Load parameters
    auto result = loadParameters();
    if (!result.success) return result;

    // Initialize device
    result = initializeDevice();
    if (!result.success) return result;

    // Setup ROS publishers
    result = setupROS();
    if (!result.success) return result;

    // Start streaming
    stream_active_ = true;
    stream_thread_ = std::thread(&FlirRos::streamingLoop, this);

    return Result<void>::ok({});
}

Result<void> FlirRos::loadParameters() {
    this->declare_parameter<std::string>("device_name", "/dev/video0");
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("camera_name", config_.camera_name);
    this->declare_parameter<std::string>("intrinsic_url", config_.intrinsic_url);
    this->declare_parameter<int>("width", config_.width);
    this->declare_parameter<int>("height", config_.height);
    this->declare_parameter<bool>("raw", config_.raw);
    this->declare_parameter<int>("gain_mode", config_.gain_mode);
    this->declare_parameter<int>("ffc_mode", config_.ffc_mode);
    this->declare_parameter<int>("use_ext_sync", config_.use_ext_sync);
    this->declare_parameter<int>("send_every_n", config_.send_every_n);
    this->declare_parameter<float>("timestamp_offset", config_.timestamp_offset);

    this->get_parameter("device_name", device_.device_path);
    this->get_parameter("serial_port", device_.serial_port);
    this->get_parameter("camera_name", config_.camera_name);
    this->get_parameter("intrinsic_url", config_.intrinsic_url);
    this->get_parameter("width", config_.width);
    this->get_parameter("height", config_.height);
    this->get_parameter("raw", config_.raw);
    this->get_parameter("gain_mode", config_.gain_mode);
    this->get_parameter("ffc_mode", config_.ffc_mode);
    this->get_parameter("use_ext_sync", config_.use_ext_sync);
    this->get_parameter("send_every_n", config_.send_every_n);
    this->get_parameter("timestamp_offset", config_.timestamp_offset);

    return Result<void>::ok({});
}

Result<void> FlirRos::initializeDevice() {
    // Resolve device path
    char device_realpath[1024];
    if (!realpath(device_.device_path.c_str(), device_realpath)) {
        return Result<void>::error("Failed to resolve device path");
    }
    device_.device_path = device_realpath;
    LOG_INFO("Device path resolved to %s", device_.device_path.c_str());

    // Open device
    device_.fd = open(device_.device_path.c_str(), O_RDWR);
    if (device_.fd < 0) {
        return Result<void>::error("Failed to open device");
    }

    // Verify streaming capabilities
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(device_.fd, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        return Result<void>::error("Device cannot stream video");
    }

    // Set format and request buffers
    if (!setFormat(device_.fd, config_.raw)) {
        return Result<void>::error("Failed to set video format");
    }
    if (!requestBuffers(device_.fd)) {
        return Result<void>::error("Failed to request buffers");
    }
    if (!startStreaming(device_.fd)) {
        return Result<void>::error("Failed to start streaming");
    }

    return Result<void>::ok({});
}

Result<void> FlirRos::setupROS() {
    // Initialize image transport and camera info manager
    publisher_.it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_.cinfo = std::make_shared<camera_info_manager::CameraInfoManager>(shared_from_this(), config_.camera_name, config_.intrinsic_url);

    // Set up topic names and frame IDs
    publisher_.camera_topic_name = config_.camera_name + "/image";
    publisher_.rect_topic_name = config_.camera_name + "/image_rect";
    publisher_.base_frame_id = config_.camera_name + "/camera_link";
    publisher_.img_opt_frame_id = config_.camera_name + "/optical_frame";

    // Advertise image topics
    publisher_.image_pub = publisher_.it->advertiseCamera(publisher_.camera_topic_name, 10);
    publisher_.rect_image_pub = publisher_.it->advertise(publisher_.rect_topic_name, 10);

    return Result<void>::ok({});
}

void FlirRos::streamingLoop() {
    LOG_INFO("Starting streaming thread");
    struct v4l2_buffer bufferinfo;
    std::memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    while (stream_active_.load()) {
        if (ioctl(device_.fd, VIDIOC_QBUF, &bufferinfo) < 0) {
            LOG_ERROR("Failed to queue buffer");
            break;
        }

        if (ioctl(device_.fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
            LOG_ERROR("Failed to dequeue buffer");
            break;
        }

        frame_count_++;
        if (frame_count_ % config_.send_every_n == 0) {
            rclcpp::Time frame_time;
            getFrameTime(frame_time);
            publishFrame(bufferinfo.bytesused, frame_time);
            publishTransforms(frame_time);
            frame_count_ = 0;
        }
    }
}

bool FlirRos::setFormat(int fd, bool raw) {
    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = config_.width;
    format.fmt.pix.height = config_.height;
    format.fmt.pix.pixelformat = raw ? V4L2_PIX_FMT_Y16 : V4L2_PIX_FMT_YUV420;

    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        LOG_ERROR("Failed to set video format");
        return false;
    }
    return true;
}

bool FlirRos::requestBuffers(int fd) {
    struct v4l2_requestbuffers req;
    std::memset(&req, 0, sizeof(req));
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 1;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        LOG_ERROR("Failed to request buffers");
        return false;
    }

    struct v4l2_buffer query_buffer;
    std::memset(&query_buffer, 0, sizeof(query_buffer));
    query_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    query_buffer.memory = V4L2_MEMORY_MMAP;
    query_buffer.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0) {
        LOG_ERROR("Failed to query buffer");
        return false;
    }

    device_.buffer = mmap(nullptr, query_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, query_buffer.m.offset);
    if (device_.buffer == MAP_FAILED) {
        LOG_ERROR("Failed to mmap buffer");
        return false;
    }

    std::memset(device_.buffer, 0, query_buffer.length);
    LOG_INFO("Allocated buffer of size %d KB", query_buffer.length / 1024);
    return true;
}

bool FlirRos::startStreaming(int fd) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        LOG_ERROR("Failed to start streaming");
        return false;
    }
    return true;
}

void FlirRos::getFrameTime(rclcpp::Time& frame_time) {
    if (config_.use_ext_sync) {
        timespec system_time;
        clock_gettime(CLOCK_REALTIME, &system_time);

        uint64_t one_tenth_nsec = 100000000;  // 10 Hz
        uint64_t system_nsec = system_time.tv_nsec;
        uint64_t trigger_nsec = system_nsec - (system_nsec % one_tenth_nsec) + static_cast<uint64_t>(config_.timestamp_offset * 1e9);

        if (trigger_nsec >= 1000000000) {
            trigger_nsec -= 1000000000;
            system_time.tv_sec += 1;
        }

        frame_time = rclcpp::Time(system_time.tv_sec, trigger_nsec);
    } else {
        frame_time = this->now();
    }
}

void FlirRos::publishFrame(uint32_t bytes_used, const rclcpp::Time& time) {
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->width = config_.width;
    img->height = config_.height;
    img->is_bigendian = false;
    img->header.stamp = time;
    img->header.frame_id = publisher_.img_opt_frame_id;

    auto cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(publisher_.cinfo->getCameraInfo());
    cam_info->header = img->header;

    if (config_.raw) {
        img->encoding = "16UC1";
        img->step = config_.width * 2;
        img->data.assign(static_cast<uint8_t*>(device_.buffer), static_cast<uint8_t*>(device_.buffer) + bytes_used);
    } else {
        // Convert YUV to RGB using OpenCV
        img->encoding = "rgb8";
        img->step = config_.width * 3;
        img->data.resize(config_.width * config_.height * 3);

        cv::Mat yuv_img(config_.height + config_.height / 2, config_.width, CV_8UC1, device_.buffer);
        cv::Mat rgb_img(config_.height, config_.width, CV_8UC3, img->data.data());
        cv::cvtColor(yuv_img, rgb_img, cv::COLOR_YUV2RGB_I420);
    }

    publisher_.image_pub.publish(img, cam_info);
}

void FlirRos::publishTransform(const rclcpp::Time& time, const geometry_msgs::msg::Vector3& trans,
                               const tf2::Quaternion& q, const std::string& from,
                               const std::string& to) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = time;
    transform_msg.header.frame_id = from;
    transform_msg.child_frame_id = to;
    transform_msg.transform.translation = trans;
    transform_msg.transform.rotation = tf2::toMsg(q);
    transform_broadcaster_.sendTransform(transform_msg);
}

void FlirRos::publishTransforms(const rclcpp::Time& time) {
    tf2::Quaternion q;
    q.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    geometry_msgs::msg::Vector3 zero_translation{};
    publishTransform(time, zero_translation, q, publisher_.base_frame_id, publisher_.img_opt_frame_id);
}

}  // namespace flir_ros_sync
