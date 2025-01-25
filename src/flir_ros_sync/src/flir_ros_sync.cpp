#include "../include/flir_ros_sync/flir_ros_sync.h"

// Standard includes
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstdlib>
#include <chrono>
#include <arpa/inet.h> 
#include <chrono>

// V4L2 includes
#include <linux/videodev2.h>

// ROS includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Project includes
#include "rawBoson.h"

namespace flir_ros_sync {

// buffer and telemetry params
size_t IMAGE_SIZE;
size_t TELEMETRY_SIZE;
size_t RAW_BUFFER_SIZE;
size_t PAGE_SIZE;
size_t ALIGNED_SIZE;
size_t TELEMETRY_OFFSET;

FlirRos::FlirRos(const rclcpp::NodeOptions& options)
    : Node("flir_ros_sync", options),
        config_{},
        device_{},
        transform_broadcaster_{this},
        stream_active_{false},
        frame_count_{0}
    {
    LOG_INFO("Constructed FLIR ROS2 Node");

    // Schedule initialize() to be called after construction is complete
    // Quick workaround to avoid calling shared_from_this() in constructor too early
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&FlirRos::initialize, this)
    );
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

    Close();
}

void FlirRos::initialize() {
    LOG_INFO("Initializing FLIR ROS2 Node");

    // Cancel timer
    timer_->cancel();

    try{
        object_detection::disable_transports(shared_from_this(), "image_transport");

        loadParameters();
        initializeDevice();
        setupROS();

        // Start streaming
        stream_active_ = true;
        stream_thread_ = std::thread(&FlirRos::streamingLoop, this);
    } catch (const std::exception& e) {
        LOG_FATAL("Initialization failed: %s", e.what());
        rclcpp::shutdown();
    }
}

void FlirRos::loadParameters() {
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
    this->declare_parameter<double>("timestamp_offset", config_.timestamp_offset);
    this->declare_parameter<int>("frame_rate", config_.frame_rate);
    this->declare_parameter<int>("ffc_interval", config_.ffc_interval_mins);

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
    this->get_parameter("frame_rate", config_.frame_rate);
    this->get_parameter("ffc_interval", config_.ffc_interval_mins);

    LOG_INFO("Loaded parameters");
    LOG_INFO("Device name: %s", device_.device_path.c_str());
    LOG_INFO("Serial port: %s", device_.serial_port.c_str());
    LOG_INFO("Camera name: %s", config_.camera_name.c_str());
    LOG_INFO("Intrinsic URL: %s", config_.intrinsic_url.c_str());
    LOG_INFO("Width: %d", config_.width);
    // LOG_INFO("Height: %d", config_.height);
    LOG_INFO("Image height: %d (plus 2 telemetry lines, total: %d)", 
             config_.height, config_.total_height);
    LOG_INFO("Raw: %s", config_.raw ? "true" : "false");
    LOG_INFO("Gain mode: %d", config_.gain_mode);
    LOG_INFO("FFC mode: %d", config_.ffc_mode);
    LOG_INFO("Use external sync: %s", config_.use_ext_sync ? "true" : "false");
    LOG_INFO("Send every N: %d", config_.send_every_n);
    LOG_INFO("Timestamp offset: %f", config_.timestamp_offset);
    LOG_INFO("Frame rate: %d", config_.frame_rate);
    LOG_INFO("FFC interval: %d minutes", config_.ffc_interval_mins);
}

void FlirRos::initializeDevice() {
    // Resolve device path
    char device_realpath[1024];
    if (!realpath(device_.device_path.c_str(), device_realpath)) {
        throw std::runtime_error("Failed to resolve device path");
    }
    device_.device_path = device_realpath;
    LOG_INFO("Device path resolved to %s", device_.device_path.c_str());

    // Open device
    device_.fd = open(device_.device_path.c_str(), O_RDWR);
    if (device_.fd < 0) {
        throw std::runtime_error("Failed to open device");
    }

    // Verify streaming capabilities
    struct v4l2_capability cap;
    std::memset(&cap, 0, sizeof(cap));
    if (ioctl(device_.fd, VIDIOC_QUERYCAP, &cap) < 0 ||
        !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw std::runtime_error("Device cannot stream video");
    }

    // Resolve serial port
    char serial_realpath[1024];
    if (!realpath(device_.serial_port.c_str(), serial_realpath)) {
        throw std::runtime_error("Failed to resolve serial port");
    }
    device_.serial_port = serial_realpath;
    LOG_INFO("Serial port resolved to %s", device_.serial_port.c_str());

    // Configure Gain and FFC modes
    set_gain_mode(config_.gain_mode, device_.serial_port);
    LOG_INFO("Verified gain mode set to %d", get_gain_mode(device_.serial_port));
    set_ffc_mode(config_.ffc_mode, device_.serial_port);
    LOG_INFO("Verified FFC mode set to %d", get_ffc_mode(device_.serial_port));
    shutter(device_.serial_port);  // essentially FFC and only works when FFC mode is 0, 1, or 3

    //Initialize FLIR SDK

    // Open device
    std::string port_name = device_.serial_port;
    // Lookup port number using a helper function using the port name
    int32_t port_num = FSLP_lookup_port_id(const_cast<char*>(port_name.c_str()), port_name.length());
    if (port_num == -1) {
        // Handle error: Port not found
        throw std::runtime_error("Invalid serial port: " + port_name);
    }
    FLR_RESULT result = Initialize(port_num, 921600);
    if (result != FLR_COMM_OK) {
        LOG_ERROR("Failed to initialize FLIR SDK. Error code: %d", result);
        throw std::runtime_error("FLIR SDK initialization failed");
    }
    LOG_INFO("FLIR SDK initialized successfully");
    
    // Set telemetry
    initializeTelemetry();

    // performFFC();
    
    // Might want to modify the following if operating in 8-bit mode
    // Initialize size variables with telemetry lines included
    IMAGE_SIZE = config_.width * config_.height * 2;        // 16-bit per pixel (Y16) for image only
    TELEMETRY_SIZE = config_.width * 2 * 2;                // Two telemetry lines
    RAW_BUFFER_SIZE = config_.width * config_.total_height * 2;  // Total buffer including telemetry
    PAGE_SIZE = sysconf(_SC_PAGE_SIZE);
    ALIGNED_SIZE = ((RAW_BUFFER_SIZE + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;
    TELEMETRY_OFFSET = IMAGE_SIZE;  // Telemetry starts after image

    // Set format and request buffers
    // The following function responsible for requesting enough buffer to handle the image and telemetry:
    if (!setFormat(device_.fd, config_.raw)) {
        throw std::runtime_error("Failed to set video format");
    }
    // If successful, the following function will request a buffer of the correct size:
    if (!requestBuffers(device_.fd)) {
        throw std::runtime_error("Failed to request buffers");
    }
    if (!startStreaming(device_.fd)) {
        throw std::runtime_error("Failed to start streaming");
    }
}

void FlirRos::setupROS() {
    // Initialize image transport and camera info manager
    publisher_.it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    publisher_.cinfo = std::make_shared<camera_info_manager::CameraInfoManager>(
        this, config_.camera_name, config_.intrinsic_url);

    // Set up topic names and frame IDs
    publisher_.camera_topic_name = config_.camera_name + "/image";
    publisher_.rect_topic_name = config_.camera_name + "/image_rect";
    publisher_.base_frame_id = config_.camera_name + "/camera_link";
    publisher_.img_opt_frame_id = config_.camera_name + "/optical_frame";

    // Advertise image topics
    publisher_.image_pub = publisher_.it->advertiseCamera(publisher_.camera_topic_name, 10);
    publisher_.rect_image_pub = publisher_.it->advertise(publisher_.rect_topic_name, 10);

    // Init timestamp publisher
    // publisher_.timestamp_pub = this->create_publisher<std_msgs::msg::UInt32>(config_.camera_name+"/telemetry_timestamp", 10);

    // Init FFC status publisher
    publisher_.ffc_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(config_.camera_name+"/ffc_status", 10);
}

void FlirRos::initializeTelemetry() {
    // Set telemetry location to END before configuring buffers
    auto result = telemetrySetLocation(FLR_TELEMETRY_LOC_BOTTOM);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to set telemetry location to END. Error code: %d", result);
        throw std::runtime_error("Failed to set telemetry location");
    }
    
    FLR_TELEMETRY_LOC_E location;
    result = telemetryGetLocation(&location);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to get telemetry location. Error code: %d", result);
        throw std::runtime_error("Failed to get telemetry location");
    }
    LOG_INFO("Telemetry location set to %d", location);

    // Enable telemetry
    result = telemetrySetState(FLR_ENABLE);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to enable telemetry. Error code: %d", result);
        throw std::runtime_error("Telemetry enable failed");
    }
    
    FLR_ENABLE_E state;
    result = telemetryGetState(&state);
    if (result == R_SUCCESS && state == FLR_ENABLE) {
        LOG_INFO("Telemetry state: ENABLED");
    } else {
        LOG_ERROR("Telemetry state is NOT ENABLED");
        throw std::runtime_error("Telemetry state is NOT ENABLED");
    }

    // Check telemetry packing (16 or 8 bit)
    FLR_TELEMETRY_PACKING_E pack;
    result = telemetryGetPacking(&pack);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to get telemetry packing. Error code: %d", result);
        throw std::runtime_error("Failed to get telemetry packing");
    }
    else {
        LOG_INFO("Telemetry packing: %d", pack);
    }
}

// void FlirRos::getFFCFrameCount(uint32_t& ffc_frame_count) {
//     auto result = bosonGetFFCFrameThreshold(&ffc_frame_count);
//     if (result != R_SUCCESS) {
//         LOG_ERROR("Failed to get FFC frame count. Error code: %d", result);
//         throw std::runtime_error("Failed to get FFC frame count");
//     }
//     else {
//         LOG_INFO("FFC frame count: %d", ffc_frame_count);
//     }
// }

void FlirRos::performFFC() {
    auto result = bosonRunFFC();
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to run FFC. Error code: %d", result);
        throw std::runtime_error("Failed to run FFC");
    }
    LOG_INFO("FFC started successfully");

    // Publish FFC status
    publishFFCStatus(true);
}

void FlirRos::checkNUCTableStatus() {
    // Verify the state of NUC table
    auto result = bosonCheckForTableSwitch();
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to check for table switch. Error code: %d", result);
        // throw std::runtime_error("Failed to check for table switch");
    }
    else {
        LOG_INFO("Checked for table switch: %d", result);
    }

    uint32_t desiredTableNumber;
    result = bosonGetDesiredTableNumber(&desiredTableNumber);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to get desired table number. Error code: %d", result);
        // throw std::runtime_error("Failed to get desired table number");
    }
    else {
        LOG_INFO("Desired table number: %d", desiredTableNumber);
    }
}

void FlirRos::getFFCStatus(int16_t& status) {
    auto result = bosonGetFFCInProgress(&status);
    if (result != R_SUCCESS) {
        LOG_ERROR("Failed to get FFC status. Error code: %d", result);
        // throw std::runtime_error("Failed to get FFC status");
    }
    else {
        // LOG_INFO("FFC status: %d", status);
    }
}

void FlirRos::publishFFCStatus(bool status) {
    auto ffc_status_msg = std::make_shared<std_msgs::msg::Bool>();
    ffc_status_msg->data = status;
    publisher_.ffc_status_pub_->publish(*ffc_status_msg);
}

void FlirRos::streamingLoop() {
    LOG_INFO("Starting streaming thread");
    struct v4l2_buffer bufferinfo;
    std::memset(&bufferinfo, 0, sizeof(bufferinfo));
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    // Initialize time tracking for FFC
    // auto last_ffc_time = std::chrono::steady_clock::now();
    bool current_ffc_status = true; // FFC happens at start (see initialization code above) and we want to force publish FFC flag (see at bottom of this function)
    // int ffc_frame_threshold = 0;

    // Counter for FFC threshold
    // uint32_t ffc_frame_count = 0;
    // getFFCFrameCount(ffc_frame_count);
    // ffc_frame_threshold = ffc_frame_count;

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

        // Check if 3 minutes have passed since last FFC
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::minutes>(now - last_ffc_time);
        if (elapsed.count() >= config_.ffc_interval_mins) // || frame_count_ == 4)  // Do FFC right after 3rd frame (1st published frame)
        { 
            // Perform FFC
            performFFC();
            current_ffc_status = true;
            // last_ffc_time = now;
            last_ffc_frame_count_ = frame_count_;

            // Verify the state of NUC table
            checkNUCTableStatus();
        }

        // Note: Publishing only after 3 frames because telemetry data is wrong in the first frame, so we skip a few frames
        if (frame_count_ % config_.send_every_n == 0 && frame_count_ > 3) {
            rclcpp::Time frame_time;
            // TELEMETRY EXTRACTION DONE HERE
            extractTelemetryTimestamp(device_.buffer, bufferinfo.bytesused, frame_time);
            publishFrame(bufferinfo.bytesused, frame_time);
            publishTransforms(frame_time);
        }

        // Keeping track of ffc status
        // auto frame_count_diff = frame_count_ - last_ffc_frame_count_;
        // if (current_ffc_status && frame_count_diff > ffc_frame_threshold) {
        //     // Publish FFC status
        //     auto ffc_status_msg = std::make_shared<std_msgs::msg::Bool>();
        //     ffc_status_msg->data = false;
        //     publisher_.ffc_status_pub_->publish(*ffc_status_msg);
        //     current_ffc_status = false;
        //     LOG_INFO("FFC completed");
        // }

        // Verify the state of FFC by directly checking the camera only if FFC has been initiated and some time has passed
        // if (current_ffc_status && frame_count_ >= last_ffc_frame_count_) {}
        int16_t status;
        getFFCStatus(status);
        current_ffc_status = status;
        publishFFCStatus(current_ffc_status);
    }
}

bool FlirRos::setFormat(int fd, bool raw) {
    struct v4l2_format format;
    std::memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = config_.width;
    format.fmt.pix.height = config_.total_height;  // Use total height including telemetry
    format.fmt.pix.pixelformat = raw ? V4L2_PIX_FMT_Y16 : V4L2_PIX_FMT_YUV420;
    format.fmt.pix.sizeimage = ALIGNED_SIZE;

    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        LOG_ERROR("Failed to set format with size %zu", ALIGNED_SIZE);
        return false;
    }

    // Verify format
    struct v4l2_format verify_format;
    std::memset(&verify_format, 0, sizeof(verify_format));
    verify_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (ioctl(fd, VIDIOC_G_FMT, &verify_format) < 0) {
        LOG_ERROR("Failed to verify format");
        return false;
    }

    if (verify_format.fmt.pix.height != config_.total_height) {
        LOG_ERROR("Failed to set correct height. Requested: %d, Got: %d",
                 config_.total_height, verify_format.fmt.pix.height);
        return false;
    }

    LOG_INFO("Format set successfully: %dx%d", 
             verify_format.fmt.pix.width, verify_format.fmt.pix.height);

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

    // Verify buffer size is sufficient
    if (query_buffer.length < IMAGE_SIZE) {
        LOG_ERROR("Buffer size too small: got %d bytes, need %zu bytes", 
                 query_buffer.length, IMAGE_SIZE);
        return false;
    }

    // Map buffer
    device_.buffer = mmap(nullptr, query_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, query_buffer.m.offset);
    if (device_.buffer == MAP_FAILED) {
        LOG_ERROR("Failed to mmap buffer");
        return false;
    }

    // Zero buffer
    std::memset(device_.buffer, 0, query_buffer.length);
    LOG_INFO("Allocated buffer of size %zu bytes (Image: %zu bytes, Telemetry: %zu bytes)", 
             query_buffer.length, IMAGE_SIZE, TELEMETRY_SIZE);
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

// void FlirRos::getFrameTime(rclcpp::Time& frame_time) {
//     if (config_.use_ext_sync) {
//         timespec system_time;
//         clock_gettime(CLOCK_REALTIME, &system_time);

//         uint64_t one_amount_nsec = 1000000000 / config_.frame_rate;  // 1 / frame_rate in ns
//         uint64_t system_nsec = system_time.tv_nsec;
//         uint64_t trigger_nsec = system_nsec - (system_nsec % one_amount_nsec) + static_cast<uint64_t>(config_.timestamp_offset * 1e9);

//         if (trigger_nsec >= 1000000000) {
//             trigger_nsec -= 1000000000;
//             system_time.tv_sec += 1;
//         }

//         frame_time = rclcpp::Time(system_time.tv_sec, trigger_nsec);
//     } else {
//         frame_time = this->now();
//     }
// }

void FlirRos::extractTelemetryTimestamp(void* buffer, size_t buffer_size, rclcpp::Time& frame_time) {
    if (buffer_size < RAW_BUFFER_SIZE) {
        LOG_ERROR("Buffer too small for telemetry: %zu < %zu", buffer_size, RAW_BUFFER_SIZE);
        // throw std::runtime_error("Buffer too small for telemetry");
    }

    // Access telemetry lines (last two lines of the buffer)
    const uint16_t* telemetry_data = static_cast<uint16_t*>(buffer) + (config_.width * config_.height);
    
    const size_t timestamp_offset = 140;  // 140th byte in the telemetry data
    // Extract timestamp (in milliseconds) from telemetry: byte 140 and 141
    uint32_t timestamp = (static_cast<uint32_t>(telemetry_data[timestamp_offset]) << 16) |
                     telemetry_data[timestamp_offset + 1];
    
    // Publish FLIR timestamp
    // std_msgs::msg::UInt32 timestamp_msg;
    // timestamp_msg.data = timestamp;
    // publisher_.timestamp_pub->publish(timestamp_msg);

    // The following code:
    // 1. Have the milliseconds from telemetry, which is internal clock since the start of sensor
    // 2. Take the current system time, and set it as system_time_init only as first timestamp (T1)
    // 3. The next time stamps (T2, T3, ... Tn) are system_time_init + (timestamp - timestamp_init) * 1e6 (convert to ns for ROS2)

    if (config_.use_ext_sync) {
        if (first_frame_) {
            timestamp_init_ = timestamp;
            system_time_init_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            // Rounding system time to the nearest 100ms
            // Add half the divisor before dividing, then multiply back.
            system_time_init_ = ((system_time_init_ + 100000000/2) / 100000000) * 100000000;
            // LOG_INFO("Initial telemetry timestamp: %u, System time: %lu", timestamp, system_time_init_);


            first_frame_ = false;
            frame_time = rclcpp::Time(system_time_init_);
        } else {
            // Calculate timestamp offset in nanoseconds
            uint64_t timestamp_offset_ns = static_cast<uint64_t>(timestamp - timestamp_init_) * 1000000; // Convert ms to ns
            uint64_t final_timestamp = system_time_init_ + timestamp_offset_ns;
            // LOG_INFO("Current telemetry: %u, Initial telemetry: %u, System time init: %lu, Offset: %lu, Final: %lu", 
            //          timestamp, timestamp_init_, system_time_init_, timestamp_offset_ns, final_timestamp);
            frame_time = rclcpp::Time(final_timestamp);
        }
    } else {
        frame_time = this->now();
    }
    
    // LOG_INFO("Extracted telemetry timestamp: %u", timestamp);

    // THE FOLLOWING WAS TRIED AND DOES NOT YIELD ANY USEFUL INFORMATION, Feel free to try yourself
    // float camera_timestamp = 0;
    // // Call bosonGetTimeStamp with desired timestamp type
    // FLR_RESULT result = bosonGetTimeStamp(FLR_BOSON_UARTINIT, &camera_timestamp);
    // if(result != R_SUCCESS) {
    //     LOG_ERROR("Failed to extract timestamp. Error code: %d", result);
    //     return;
    // }

    // LOG_INFO("Camera timestamp: %f", camera_timestamp);
    /*
    enum e_FLR_BOSON_TIMESTAMPTYPE_E {
    FLR_BOSON_UARTINIT = (int32_t) 0,
    FLR_BOSON_PIXELCLOCKINIT = (int32_t) 1,
    FLR_BOSON_AUTHEVENT = (int32_t) 2,
    FLR_BOSON_FIRSTVALIDIMAGE = (int32_t) 3,
    FLR_BOSON_TIMESTAMPTYPE_END = (int32_t) 4,
    };*/
}

void FlirRos::publishFrame(uint32_t bytes_used, const rclcpp::Time& time) {
    auto img = std::make_shared<sensor_msgs::msg::Image>();
    img->width = config_.width;
    img->height = config_.height;  // Use original height without telemetry
    img->is_bigendian = false;
    img->header.stamp = time;
    img->header.frame_id = publisher_.img_opt_frame_id;

    auto cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(publisher_.cinfo->getCameraInfo());
    cam_info->header = img->header;

    if (config_.raw) {
        img->encoding = "16UC1";
        img->step = config_.width * 2;
        // Copy only the image data, excluding telemetry lines
        img->data.assign(
            static_cast<uint8_t*>(device_.buffer),
            static_cast<uint8_t*>(device_.buffer) + IMAGE_SIZE
        );

        // Extract telemetry from the additional lines, done here????
        // extractTelemetryTimestamp(device_.buffer, bytes_used, time);
    } else {
        // Handle YUV conversion (excluding telemetry lines)
        img->encoding = "rgb8";
        img->step = config_.width * 3;
        img->data.resize(config_.width * config_.height * 3);

        cv::Mat yuv_img(config_.height, config_.width, CV_8UC1, device_.buffer);
        cv::Mat rgb_img(config_.height, config_.width, CV_8UC3, img->data.data());
        cv::cvtColor(yuv_img, rgb_img, cv::COLOR_YUV2RGB_I420);
    }

    object_detection::publish_if_subscribed(publisher_.image_pub, img, cam_info);

    // Publish rectified image
    sensor_msgs::msg::Image::SharedPtr rect_msg = rectify_image(img, cam_info);
    object_detection::publish_if_subscribed(publisher_.rect_image_pub, rect_msg);

    // Publish transform
    // geometry_msgs::msg::Vector3 translation;
    // tf2::Quaternion rotation;
    // publishTransforms...
}

sensor_msgs::msg::Image::SharedPtr FlirRos::rectify_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info)
{
    // update camera info
    cam_model_.fromCameraInfo(cam_info);
    // get the image to cv format
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return nullptr;
    }

    if (!maps_initialized_) {
        // Get camera matrix and distortion coefficients
        cv::Mat K = cv::Mat(cam_model_.intrinsicMatrix());
        cv::Mat D = cv::Mat(cam_model_.distortionCoeffs());

        // Calculate optimal new camera matrix
        cv::Mat P = cv::getOptimalNewCameraMatrix(
            K, D, 
            cv::Size(image_msg->width, image_msg->height),
            1.0,  // alpha=1.0 to keep all pixels
            cv::Size(image_msg->width, image_msg->height),
            nullptr, false);
        
        // Initialize undistortion maps
        cv::initUndistortRectifyMap(
            K, D, cv::Mat(), 
            P,
            cv::Size(image_msg->width, image_msg->height),
            CV_32FC1,
            map1_, map2_);

        // Set flag
        maps_initialized_ = true;
    }

    // Rectify image using computed maps
    cv::Mat rect_image;
    cv::remap(cv_ptr->image, rect_image, 
              map1_, map2_, 
              cv::INTER_LINEAR,
              cv::BORDER_CONSTANT);

    // Publish the rectified image
    const sensor_msgs::msg::Image::SharedPtr rect_msg =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
            .toImageMsg();

    return rect_msg;

    // Original code:
    // cv::Mat rect_image;

    // // rectify the image
    // cam_model_.rectifyImage(cv_ptr->image, rect_image, CV_INTER_AREA);  // changed from CV_INTER_LINEAR to CV_INTER_AREA

    // // publish the rectified image
    // const sensor_msgs::msg::Image::SharedPtr rect_msg =
    //     cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
    //         .toImageMsg();

    // Yifei's code:
    // def rectify_image(image, intrinsics, distortion):
    // fx, fy, cx, cy = intrinsics
    // camera_matrix = np.array([[fx, 0, cx],
    //                         [0, fy, cy],
    //                         [0,  0,  1]])
    // distortion_coeffs = np.array(distortion)
    // height, width = image.shape[:2]
    // # new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 1, (width, height))
    // # rectified_image = cv2.undistort(image, camera_matrix, distortion_coeffs, None, new_camera_matrix)
    // rectified_image = cv2.undistort(image, camera_matrix, distortion_coeffs)

    // # x, y, w, h = roi
    // # if w > 0 and h > 0:
    // #     rectified_image = rectified_image[y:y+h, x:x+w]
    
    // return rectified_image
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

RCLCPP_COMPONENTS_REGISTER_NODE(flir_ros_sync::FlirRos)
