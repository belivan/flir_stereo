#include "../include/flir_ros_sync.h"
#include <asm/types.h>
#include <fcntl.h>
//#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/image_encodings.hpp>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h> // for char *realpath(const char *restrict path, char *restrict resolved_path); to read symlink for thermal serial
#include <chrono>
#include "rawBoson.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define LOG_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)
#define LOG_INFO_STREAM(args...) RCLCPP_INFO_STREAM(this->get_logger(), args)
#define LOG_FATAL(...) RCLCPP_FATAL(this->get_logger(), __VA_ARGS__)
typedef rclcpp::Time RosTime;  // not used really


// https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/capture.c.html
// http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define CHECK_FATAL(x, err)    \
  if ((x))                     \
  {                            \
    LOG_FATAL_STREAM(err); \
    return;                    \
  }

#if defined(IS_ROS1)
PLUGINLIB_EXPORT_CLASS(flir_ros_sync::FlirRos, LOG::LOG)
#elif defined(IS_ROS2)
RCLCPP_COMPONENTS_REGISTER_NODE(flir_ros_sync::FlirRos)
#endif

namespace flir_ros_sync
{
  std::shared_ptr<FlirRos> FlirRos::onInit(const rclcpp::NodeOptions & options)
  {
      // TODO: This might not work at all
      auto node = std::make_shared<FlirRos>(options);
      // Start the stream thread
      node->stream_thread = std::thread(&FlirRos::streaming_loop, node);
      return node;
  }

  FlirRos::FlirRos(const rclcpp::NodeOptions & options)
  : Node("flir_ros_sync", options)
  {
    LOG_INFO("Initializing FLIR ROS2 Node");

    // unpack device_name symlink
    this->declare_parameter<std::string>("device_name", "/dev/video0");
    std::string deviceName;
    this->get_parameter("device_name", deviceName);

    char deviceNameRoot[1024];
    char *deviceResult = realpath(deviceName.c_str(), deviceNameRoot);
    CHECK_FATAL(!deviceResult, "Serial port " << deviceName << " cannot be resolved!");
    LOG_INFO_STREAM("Serial port " << deviceName << " resolved to " << deviceNameRoot);
    
    // TODO: get_ros_param(); // get the ros paramters from launch file
    get_ros_param(); // get the ros paramters from launch file

    // 1. Try to open the device
    fd.reset(open(deviceName.c_str(), O_RDWR));
    CHECK_FATAL(fd < 0, "Unable to open device" << deviceNameRoot);

    // 2. Verify that the camera can actually stream
    struct v4l2_capability cap;
    CLEAR(cap);
    CHECK_FATAL(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0,
                "Unable to query capabilities!");
    CHECK_FATAL(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE),
                "Device can't stream video!");

    // 3. Set the device format (default is raw)
    this->declare_parameter<int>("publish_image_sharing_every_n", publish_image_sharing_every_n);
    this->declare_parameter<bool>("raw", true);

    this->get_parameter("publish_image_sharing_every_n", publish_image_sharing_every_n);
    this->get_parameter("raw", raw);

    CHECK_FATAL(!set_format(fd, raw), "Unable to set video format!");

    // 4. unpack serial_port symlink
    this->declare_parameter<std::string>("serial_port");
    std::string serialPort;
    this->get_parameter("serial_port", serialPort);
    char serialPortRoot[1024];
    char *serialResult = realpath(serialPort.c_str(), serialPortRoot);
    CHECK_FATAL(!serialResult, "Serial port " << serialPort << " cannot be resolved!");
    LOG_INFO_STREAM("Serial port " << serialPort << " resolved to " << serialPortRoot);

    // 6. set gain mode and FFC(flat field correction, regarding image global illumination) mode
    this->declare_parameter<int>("gain_mode", gain_mode);
    this->declare_parameter<int>("ffc_mode", ffc_mode);
    this->get_parameter("gain_mode", gain_mode);
    this->get_parameter("ffc_mode", ffc_mode);

    set_gain_mode(gain_mode, serialPortRoot);
    set_ffc_mode(ffc_mode, serialPortRoot);
    shutter(serialPortRoot); // Best practice: set FFC mode to manual and do FFC only once during initialization

    // 7. get signed timestamp offset in seconds, true capture time = message receival time + offset, should be negative if message arrive later than capture
    this->declare_parameter<float>("timestamp_offset", timestampOffset);
    this->get_parameter("timestamp_offset", timestampOffset);

    // 8. Initialize the buffers (mmap)
    CHECK_FATAL(!request_buffers(fd), "Requesting buffers failed!");

    // 9. Start streaming
    CHECK_FATAL(!start_streaming(fd), "Stream start failed!");

    LOG_INFO("Ready to start streaming camera!");
    stream = true;

    // 10. Set up ROS publishers, now that we're confident we can stream.
    // TODO: setup_ros();
    setup_ros();
  }

  FlirRos::~FlirRos()
  {
    LOG_INFO("Destructor called");

    stream.store(false);
    if (stream_thread.joinable())
    {
      stream_thread.join();
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    CHECK_FATAL(ioctl(fd, VIDIOC_STREAMOFF, &type) < 0,
                "Couldn't properly close the stream!");
    }

#endif


  void FlirRos::streaming_loop()
  {
    LOG_INFO("Starting thread!");
    struct v4l2_buffer bufferinfo;
    CLEAR(bufferinfo);
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    // FFC trigger time
    // auto last_ffc_time = std::chrono::steady_clock::now();

    while (stream.load()) {
      if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
        LOG_INFO("Couldn't queue :(");
        LOG_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't queue anymore
      }

      if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
        LOG_INFO("Couldn't dequeue :(");
        LOG_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't de-queue
      }

      // UPDATE: Check if 3 minutes have passed since the last FFC
      // auto now = std::chrono::steady_clock::now();
      // if (std::chrono::duration_cast<std::chrono::seconds>(now - last_ffc_time).count() >= 30) {
      //     // Trigger FFC
      //     shutter(serialPortRoot);
      //     LOG_INFO("FFC triggered");

      //     // Update the last FFC trigger time
      //     last_ffc_time = now;
      // }

      // TODO(vasua): Publish diagnostic msgs here.

      count=count+1;
      if(count%send_every_n==0)
      { 
        rclcpp::Time frame_time; // note: ros::Time depends on ROS version
        get_frame_time(frame_time);

        publish_frame(bufferinfo.bytesused, frame_time);
        count=0;
        // publish tf for every frame
        publish_transforms(frame_time);
      }
      
    }
  }

  bool FlirRos::set_format(int fd, bool raw)
  {
    struct v4l2_format format;
    CLEAR(format);
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (raw)
    {
      format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16; // Y16
    }
    else
    {
      format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420; // YU12
    }

    format.fmt.pix.width = width;
    format.fmt.pix.height = height;

    // Actually set the video mode
    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0)
    {
      LOG_ERROR("VIDIOC_S_FMT");
      LOG_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    return true;
  }

  bool FlirRos::request_buffers(int fd)
  {
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 1;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0)
    {
      LOG_ERROR("VIDIOC_REQBUFS");
      LOG_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    LOG_INFO_STREAM("Device requested " << req.count << " buffers!");
    // The device only requests a single buffer in both cases.

    struct v4l2_buffer query_buffer;
    CLEAR(query_buffer);
    query_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    query_buffer.memory = V4L2_MEMORY_MMAP;
    query_buffer.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0)
    {
      LOG_ERROR("VIDIOC_QUERYBUF");
      LOG_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    buffer = mmap(NULL,                                        /* start anywhere */
                  query_buffer.length, PROT_READ | PROT_WRITE, /* required */
                  MAP_SHARED,                                  /* recommended */
                  fd, query_buffer.m.offset);

    if (buffer == MAP_FAILED)
    {
      LOG_ERROR("MAP_FAILED");
      LOG_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    memset(buffer, 0, query_buffer.length);
    LOG_INFO("Allocated buffer of size %d KB", query_buffer.length / 1024);
    return true;
  }

  bool FlirRos::start_streaming(int fd)
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
      LOG_ERROR("VIDIOC_STREAMON");
      LOG_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    return true;
  }

  void FlirRos::setup_ros_names()
  {
    camera_topic_name = camera_name + "/image";
    rect_topic_name = camera_name + "/image_rect";
    base_frame_id = camera_name + "/camera_link";
    img_opt_frame_id = camera_name + "/optical_frame";
  }

  void FlirRos::get_ros_param()
  {
  #if defined(IS_ROS1)
    private_nh.getParam("camera_name", camera_name);
    private_nh.getParam("intrinsic_url", intrinsic_url);
    private_nh.getParam("width", width);
    private_nh.getParam("height", height);
    private_nh.getParam("use_ext_sync", use_ext_sync);
    private_nh.getParam("send_every_n", send_every_n);
  #elif defined(IS_ROS2)
    this->declare_parameter<std::string>("camera_name", "flir");
    this->declare_parameter<std::string>("intrinsic_url", "package://flir_ros_sync/config/flir_intrinsics.yaml");
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 512);
    this->declare_parameter<int>("use_ext_sync", 1);
    this->declare_parameter<int>("send_every_n", 1);

    this->get_parameter("camera_name", camera_name);
    this->get_parameter("intrinsic_url", intrinsic_url);
    this->get_parameter("width", width);
    this->get_parameter("height", height);
    this->get_parameter("use_ext_sync", use_ext_sync);
    this->get_parameter("send_every_n", send_every_n);
  #endif
  }

  void FlirRos::setup_ros()
  {
    setup_ros_names();

  #if defined(IS_ROS1)
    it.reset(new image_transport::ImageTransport(nh));
    cinfo.reset(new camera_info_manager::CameraInfoManager(nh));

    object_detection::disable_transports(&nh, camera_topic_name);
    object_detection::disable_transports(&nh, rect_topic_name);
  
  #elif defined(IS_ROS2)
    it = std::make_shared<image_transport::ImageTransport>(shared_from_this());
    cinfo = std::make_shared<camera_info_manager::CameraInfoManager>(shared_from_this());

    // TODO: update the function to use ROS2
    object_detection::disable_transports(shared_from_this(), camera_topic_name);
    object_detection::disable_transports(shared_from_this(), rect_topic_name);
  #endif

    // load camera intrinsics paramters to camera_info_manager
    cinfo->setCameraName(camera_name);
    cinfo->loadCameraInfo(intrinsic_url);
  
    image_pub = it->advertiseCamera(camera_topic_name, 10);
    rect_image_pub = it->advertise(rect_topic_name, 10);

    // std::vector<std::string> topic_names;
    // topic_names.push_back(("nv_" + camera_name));
    // nv_image_pub = new nv2ros::Publisher(nh, topic_names);  // not used
  }

  sensor_msgs::msg::Image::SharedPtr FlirRos::rectify_image(
      const sensor_msgs::msg::Image::ConsSharedtPtr& image_msg,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_ptr)
  {
    // update camera info
    cam_model.fromCameraInfo(cam_info_ptr);
    // get the image to cv format
    const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat rect_image;

    // rectify the image
    cam_model.rectifyImage(image, rect_image, CV_INTER_LINEAR);

    // publish the rectified image
    sensor_msgs::msg::Image::SharedPtr rect_msg =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
            .toImageMsg();
    return rect_msg;
  }

  void FlirRos::get_frame_time(rclcpp::Time &frame_time)
  {
    if (use_ext_sync)
    {
      timespec systemtime;
      clock_gettime(CLOCK_REALTIME, &systemtime);
      // take the 1/60s second since we are trigger the camera at 60hz
      // and the trigger is aligned with system time (CLOCK_REALTIME)

      uint32_t one_tenth_nsec = 100000000;  // MODIFIED TO 10 HZ from one_sixtieth_nsec = 16666667;
      time_t system_sec = systemtime.tv_sec;
      time_t system_nsec = systemtime.tv_nsec;
      uint32_t trigger_nsec = static_cast<uint32_t>(system_nsec) - (static_cast<uint32_t>(system_nsec) % one_tenth_nsec) + static_cast<uint32_t>(timestampOffset * 1e9);

      // Handle potential overflow of nanoseconds
      if (trigger_nsec >= 1000000000) {
        trigger_nsec -= 1000000000;
        system_sec += 1;
      }

      // changed from uint32_t to uint64_t
      frame_time = rclcpp::Time(static_cast<uint64_t>(system_sec), 
                                static_cast<uint64_t>(trigger_nsec));
    }
    else
    {
      frame_time = this->now();
    }
  }

  void FlirRos::publish_frame(uint32_t bytes_used, rclcpp::Time time)
  {
    sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image);
    img->width = width;
    img->height = height;
    img->is_bigendian = false; // hopefully
    img->header.stamp = time;
    img->header.frame_id = img_opt_frame_id;

    // get current camera info data and set header
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info_ptr(new sensor_msgs::msg::CameraInfo(cinfo->getCameraInfo()));
    cam_info_ptr->header.frame_id = img_opt_frame_id;
    cam_info_ptr->header.stamp = img->header.stamp;

    if (raw)
    {                        // Can publish image directly, don't need opencv.
      img->step = width * 2; // 2 bytes per pixel
      // img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      img->encoding = "16UC1";
      img->data.insert(img->data.begin(), reinterpret_cast<char *>(buffer),
                       reinterpret_cast<char *>(buffer) + bytes_used);

      /*if (publish_image_sharing_every_n < 1000)
      {
        static nv2ros::NvImageMessage nv_image_message(width, height, time);
        static float prev_p_min = 100000;
        static float prev_p_max = 0;
        static int count = 0;
        count++;
        if (count == publish_image_sharing_every_n)
        {
          count = 0;

          int pitch = nv_image_message.get_pitch();
          void *data;
          nv_image_message.mem_map(&data);
          nv_image_message.mem_sync_for_cpu(&data);

          uint16_t *image_pixels = (uint16_t *)buffer;
          float p_min = 1000000000.f;
          float p_max = 0;
          for (int w = 0; w < width; w++)
          {
            for (int h = 0; h < height; h++)
            {
              float p = image_pixels[h * width + w];
              p_min = std::min(p, p_min);
              p_max = std::max(p, p_max);
              p = (p - prev_p_min) / std::max(prev_p_max - prev_p_min, 0.00001f);
              p = std::max(0.f, std::min(1.f, p));
              uint8_t p_int = p * 255.f;

              ((uint8_t *)(data))[h * pitch + w * 4] = p_int;
              ((uint8_t *)(data))[h * pitch + w * 4 + 1] = p_int;
              ((uint8_t *)(data))[h * pitch + w * 4 + 2] = p_int;
              ((uint8_t *)(data))[h * pitch + w * 4 + 3] = 255;
            }
          }
          prev_p_min = p_min;
          prev_p_max = p_max;
          // ROS_INFO_STREAM("min max: " << p_min << " " << p_max);

          nv_image_message.mem_sync_for_device(&data);
          nv_image_message.mem_unmap(&data);

          nv_image_message.set_stamp(time);
          nv_image_pub->publish(nv_image_message);
        }
      }*/
    }
    else
    { // Use opencv to convert from YUV420 to RGB
      // There might be a way to get rid of the initialization done inside
      // resize() if we're clever, but probably not necessary.
      auto bytes_per_pixel = 3;
      img->data.resize(width * height * bytes_per_pixel);

      auto luma_height = height + height / 2; // 3/2 * height
      auto luma_width = width;

      // Directly use the input and output buffers with Mat as a wrapper,
      // so that we don't have any unnecessary copies.
      cv::Mat luma(luma_height, luma_width, CV_8UC1, buffer);
      cv::Mat rgb(height, width, CV_8UC3, &img->data[0]);
      cv::cvtColor(luma, rgb, cv::COLOR_YUV2RGB_I420);

      img->step = width * bytes_per_pixel;
      // img->encoding = sensor_msgs::image_encodings::RGB8;
      img->encoding = "rgb8";

      /*if (publish_image_sharing_every_n < 1000)
      {
        static nv2ros::NvImageMessage nv_image_message(width, height, time);
        static int count = 0;
        count++;
        if (count == publish_image_sharing_every_n)
        {
          count = 0;

          int pitch = nv_image_message.get_pitch();
          void *data;
          nv_image_message.mem_map(&data);
          nv_image_message.mem_sync_for_cpu(&data);

          std::vector<uint8_t> const &pixelArray = img->data;
          for (int w = 0; w < width; w++)
          {
            for (int h = 0; h < height; h++)
            {
              const int currentPixelIndex = (h * width + w) * bytes_per_pixel;
              ((uint8_t *)(data))[h * pitch + w * 4] = pixelArray[currentPixelIndex];         // red
              ((uint8_t *)(data))[h * pitch + w * 4 + 1] = pixelArray[currentPixelIndex + 1]; // green
              ((uint8_t *)(data))[h * pitch + w * 4 + 2] = pixelArray[currentPixelIndex + 2]; // blue
              ((uint8_t *)(data))[h * pitch + w * 4 + 3] = 255;                              // alpha
            }
          }

          nv_image_message.mem_sync_for_device(&data);
          nv_image_message.mem_unmap(&data);

          nv_image_message.set_stamp(time);
          nv_image_pub->publish(nv_image_message);
        }
      }*/
    }
    // publish raw image
    image_pub.publish(img, cam_info_ptr);
    // rectify and publish rectified image
    // sensor_msgs::ImagePtr rect_msg = rectify_image(img, cam_info_ptr);
    // rect_image_pub.publish(rect_msg);
  }

  void FlirRos::publish_transform(const rclcpp::Time &time, const tf::Vector3 &trans,
                                  const tf2::Quaternion &q,
                                  const std::string &from,
                                  const std::string &to)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = time;
    msg.header.frame_id = from;
    msg.child_frame_id = to;

    msg.transform.translation = tf2::toMsg(trans);
    msg.transform.rotation = tf2::toMsg(q);
    br.sendTransform(msg);
  }

  void FlirRos::publish_transforms(const rclcpp::Time &time)
  {
    tf2::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    const geometry_msgs::msg::Vector3 zero_translation{0, 0, 0};

    // Set camera link as Z upward, X to front and Y to left
    // Set optical frame as Z forward, X to right and Y downward
    publish_transform(time, zero_translation, quaternion_optical, base_frame_id,
                      img_opt_frame_id);
  }
} // namespace flir_ros_sync
