#include "../include/flir_ros_sync.h"
#include <asm/types.h>
#include <fcntl.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h> // for char *realpath(const char *restrict path, char *restrict resolved_path); to read symlink for thermal serial
#include <chrono>
#include "rawBoson.h"

// https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/capture.c.html
// http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define CHECK_FATAL(x, err)    \
  if ((x))                     \
  {                            \
    NODELET_FATAL_STREAM(err); \
    return;                    \
  }

PLUGINLIB_EXPORT_CLASS(flir_ros_sync::FlirRos, nodelet::Nodelet)

namespace flir_ros_sync
{
  void FlirRos::onInit()
  {
    NODELET_INFO("IInitializing flir ros node");

    // get ros node handle
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    // unpack device_name symlink
    std::string deviceName;
    private_nh.getParam("device_name", deviceName);
    char deviceNameRoot[1024];
    char *deviceResult = realpath(deviceName.c_str(), deviceNameRoot);
    CHECK_FATAL(!deviceResult, "Serial port " << deviceName << " cannot be resolved!");
    NODELET_INFO_STREAM("Serial port " << deviceName << " resolved to " << deviceNameRoot);
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
    private_nh.param("publish_image_sharing_every_n", publish_image_sharing_every_n, publish_image_sharing_every_n);
    private_nh.param("raw", raw, raw);
    CHECK_FATAL(!set_format(fd, raw), "Unable to set video format!");

    // 4. unpack serial_port symlink
    std::string serialPort;
    private_nh.getParam("serial_port", serialPort);
    char serialPortRoot[1024];
    char *serialResult = realpath(serialPort.c_str(), serialPortRoot);
    CHECK_FATAL(!serialResult, "Serial port " << serialPort << " cannot be resolved!");
    NODELET_INFO_STREAM("Serial port " << serialPort << " resolved to " << serialPortRoot);

    // 6. set gain mode and FFC(flat field correction, regarding image global illumination) mode
    private_nh.param("gain_mode", gain_mode, gain_mode);
    private_nh.param("ffc_mode", ffc_mode, ffc_mode);
    set_gain_mode(gain_mode, serialPortRoot);
    set_ffc_mode(ffc_mode, serialPortRoot);
    shutter(serialPortRoot); // Best practice: set FFC mode to manual and do FFC only once during initialization

    // 7. get signed timestamp offset in seconds, true capture time = message receival time + offset, should be negative if message arrive later than capture
    private_nh.param("timestamp_offset", timestampOffset, timestampOffset);

    // 8. Initialize the buffers (mmap)
    CHECK_FATAL(!request_buffers(fd), "Requesting buffers failed!");

    // 9. Start streaming
    CHECK_FATAL(!start_streaming(fd), "Stream start failed!");

    NODELET_INFO("Ready to start streaming camera!");
    stream = true;

    // 10. Set up ROS publishers, now that we're confident we can stream.
    setup_ros();

    stream_thread = std::thread([&]()
                                {
    NODELET_INFO("Starting thread!");
    struct v4l2_buffer bufferinfo;
    CLEAR(bufferinfo);
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;

    while (stream.load()) {
      if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
        NODELET_INFO("Couldn't queue :(");
        NODELET_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't queue anymore
      }

      if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
        NODELET_INFO("Couldn't dequeue :(");
        NODELET_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't de-queue
      }

      // TODO(vasua): Publish diagnostic msgs here.

        count=count+1;
      if(count%send_every_n==0)
      { 
        ros::Time frame_time;
        get_frame_time(frame_time);
        publish_frame(bufferinfo.bytesused, frame_time);
        count=0;
        // publish tf for every frame
        publish_transforms(frame_time);
      }
      
    } });
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
      NODELET_ERROR("VIDIOC_S_FMT");
      NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
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
      NODELET_ERROR("VIDIOC_REQBUFS");
      NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    NODELET_INFO_STREAM("Device requested " << req.count << " buffers!");
    // The device only requests a single buffer in both cases.

    struct v4l2_buffer query_buffer;
    CLEAR(query_buffer);
    query_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    query_buffer.memory = V4L2_MEMORY_MMAP;
    query_buffer.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0)
    {
      NODELET_ERROR("VIDIOC_QUERYBUF");
      NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    buffer = mmap(NULL,                                        /* start anywhere */
                  query_buffer.length, PROT_READ | PROT_WRITE, /* required */
                  MAP_SHARED,                                  /* recommended */
                  fd, query_buffer.m.offset);

    if (buffer == MAP_FAILED)
    {
      NODELET_ERROR("MAP_FAILED");
      NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
      return false;
    }

    memset(buffer, 0, query_buffer.length);
    NODELET_INFO("Allocated buffer of size %d KB", query_buffer.length / 1024);
    return true;
  }

  bool FlirRos::start_streaming(int fd)
  {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
      NODELET_ERROR("VIDIOC_STREAMON");
      NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
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
    private_nh.getParam("camera_name", camera_name);
    private_nh.getParam("intrinsic_url", intrinsic_url);
    private_nh.getParam("width", width);
    private_nh.getParam("height", height);
    private_nh.getParam("use_ext_sync", use_ext_sync);
    private_nh.getParam("send_every_n", send_every_n);
  }

  void FlirRos::setup_ros()
  {
    setup_ros_names();

    it.reset(new image_transport::ImageTransport(nh));
    cinfo.reset(new camera_info_manager::CameraInfoManager(nh));

    // load camera intrinsics paramters to camera_info_manager
    cinfo->setCameraName(camera_name);
    cinfo->loadCameraInfo(intrinsic_url);

    object_detection::disable_transports(&nh, camera_topic_name);
    object_detection::disable_transports(&nh, rect_topic_name);
    image_pub = it->advertiseCamera(camera_topic_name, 1);
    rect_image_pub = it->advertise(rect_topic_name, 1);
    std::vector<std::string> topic_names;
    topic_names.push_back(("nv_" + camera_name));
    nv_image_pub = new nv2ros::Publisher(nh, topic_names);
  }

  sensor_msgs::ImagePtr FlirRos::rectify_image(
      const sensor_msgs::ImageConstPtr &image_msg,
      const sensor_msgs::CameraInfoConstPtr &cam_info_ptr)
  {
    // update camera info
    cam_model.fromCameraInfo(cam_info_ptr);
    // get the image to cv format
    const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat rect_image;

    // rectify the image
    cam_model.rectifyImage(image, rect_image, CV_INTER_LINEAR);

    // publish the rectified image
    sensor_msgs::ImagePtr rect_msg =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
            .toImageMsg();
    return rect_msg;
  }

  void FlirRos::get_frame_time(ros::Time &frame_time)
  {
    if (use_ext_sync)
    {
      timespec systemtime;
      clock_gettime(CLOCK_REALTIME, &systemtime);
      // take the 1/60s second since we are trigger the camera at 60hz
      // and the trigger is aligned with system time (CLOCK_REALTIME)
      uint32_t one_sixtieth_nsec = 16666667;
      time_t system_sec = systemtime.tv_sec;
      time_t system_nsec = systemtime.tv_nsec;
      uint32_t trigger_nsec = static_cast<uint32_t>(system_nsec) - (static_cast<uint32_t>(system_nsec) % one_sixtieth_nsec) + static_cast<uint32_t>(timestampOffset * 1e9);
      frame_time = ros::Time(static_cast<uint32_t>(system_sec), static_cast<uint32_t>(trigger_nsec));
    }
    else
    {
      frame_time = ros::Time::now();
    }
  }

  void FlirRos::publish_frame(uint32_t bytes_used, ros::Time time)
  {
    sensor_msgs::ImagePtr img(new sensor_msgs::Image);
    img->width = width;
    img->height = height;
    img->is_bigendian = false; // hopefully
    img->header.stamp = time;
    img->header.frame_id = img_opt_frame_id;

    // get current camera info data and set header
    sensor_msgs::CameraInfoPtr cam_info_ptr(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
    cam_info_ptr->header.frame_id = img_opt_frame_id;
    cam_info_ptr->header.stamp = img->header.stamp;

    if (raw)
    {                        // Can publish image directly, don't need opencv.
      img->step = width * 2; // 2 bytes per pixel
      img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      img->data.insert(img->data.begin(), reinterpret_cast<char *>(buffer),
                       reinterpret_cast<char *>(buffer) + bytes_used);

      if (publish_image_sharing_every_n < 1000)
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
      }
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
      img->encoding = sensor_msgs::image_encodings::RGB8;
      if (publish_image_sharing_every_n < 1000)
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
      }
    }
    // publish raw image
    image_pub.publish(img, cam_info_ptr);
    // rectify and publish rectified image
    // sensor_msgs::ImagePtr rect_msg = rectify_image(img, cam_info_ptr);
    // rect_image_pub.publish(rect_msg);
  }

  void FlirRos::publish_transform(const ros::Time &time, const tf::Vector3 &trans,
                                  const tf::Quaternion &q,
                                  const std::string &from,
                                  const std::string &to)
  {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = time;
    msg.header.frame_id = from;
    msg.child_frame_id = to;

    tf::vector3TFToMsg(trans, msg.transform.translation);
    tf::quaternionTFToMsg(q, msg.transform.rotation);
    br.sendTransform(msg);
  }

  void FlirRos::publish_transforms(const ros::Time &time)
  {
    tf::Quaternion quaternion_optical;
    quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    const tf::Vector3 zero_translation{0, 0, 0};

    // Set camera link as Z upward, X to front and Y to left
    // Set optical frame as Z forward, X to right and Y downward
    publish_transform(time, zero_translation, quaternion_optical, base_frame_id,
                      img_opt_frame_id);
  }

  FlirRos::~FlirRos()
  {
    NODELET_INFO("Destructor called");

    if (stream)
    {
      stream = false;
      stream_thread.join();

      enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      CHECK_FATAL(ioctl(fd, VIDIOC_STREAMOFF, &type) < 0,
                  "Couldn't properly close the stream!");
    }
  }
} // namespace flir_ros_sync
