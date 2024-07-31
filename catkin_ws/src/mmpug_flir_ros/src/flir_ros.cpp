#include "../include/flir_ros.h"
#include <asm/types.h>
#include <fcntl.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <common/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/capture.c.html
// http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define CHECK_FATAL(x, err)    \
  if ((x)) {                   \
    NODELET_FATAL_STREAM(err); \
    stream.store(false);       \
    return;                    \
  }

PLUGINLIB_EXPORT_CLASS(flir_ros::FlirRos, nodelet::Nodelet)

namespace flir_ros {
void FlirRos::onInit() {
  NODELET_INFO("Initializing flir ros node");

  // get ros node handle
  nh = getNodeHandle();
  private_nh = getPrivateNodeHandle();
  get_ros_param();  // get the ros paramters from launch file

  timer.set_name(getName());
  timer.tic("init_camera");

  // 1. Try to open the device
  fd.reset(open(device_name.c_str(), O_RDWR));
  CHECK_FATAL(fd < 0, "Unable to open device" << device_name);

  // 2. Verify that the camera can actually stream
  struct v4l2_capability cap;
  CLEAR(cap);
  CHECK_FATAL(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0,
              "Unable to query capabilities!");
  CHECK_FATAL(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE),
              "Device can't stream video!");

  // 3. Set the device format (default is raw)
  private_nh.param("raw", raw, raw);
  CHECK_FATAL(!set_format(fd, raw), "Unable to set video format!");

  // 4. Initialize the buffers (mmap)
  CHECK_FATAL(!request_buffers(fd), "Requesting buffers failed!");

  // 5. Start streaming
  CHECK_FATAL(!start_streaming(fd), "Stream start failed!");

  NODELET_INFO("Ready to start streaming camera!");
  stream = true;

  timer.toc("init_camera");

  // 6. Set up ROS publishers, now that we're confident we can stream.
  setup_ros();

  stream_thread = std::thread([&]() {
    timer.tic("thread_init");
    NODELET_INFO("Starting thread!");
    struct v4l2_buffer bufferinfo;
    CLEAR(bufferinfo);
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = 0;
    timer.toc("thread_init");

    while (stream.load()) {
      thread_start = ros::Time::now();
      timer.tic("queue");
      if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
        NODELET_INFO("Couldn't queue :(");
        NODELET_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't queue anymore
      }
      timer.toc("queue");

      timer.tic("dequeue");
      if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
        NODELET_INFO("Couldn't dequeue :(");
        NODELET_INFO_STREAM("Error: " << std::string(strerror(errno)));
        break;  // Couldn't de-queue
      }
      timer.toc("dequeue");

      if (++count != send_every_n) {
        continue;
      }
      count = 0;

      timer.tic("publish_frame");
      // TODO(vasua): Publish diagnostic msgs here.
      ros::Time time = ros::Time::now();
      publish_frame(bufferinfo.bytesused, time);
      publish_transforms(time);
      timer.toc("publish_frame");
      NODELET_INFO_STREAM_THROTTLE(2 , "Loop Time: " << (ros::Time::now() - thread_start).toSec());
    }
  });
}


bool FlirRos::set_format(int fd, bool raw) {
  struct v4l2_format format;
  CLEAR(format);
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // Anton added new format support because previous ones did not seem to work!
  // It doesn't seem to be the format lets try adjust width and height. Nope...
  if (raw) {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;  // Y16
    // format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;  // YU12
  } else {
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;  // YU12
    // format.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;  // NV12
  }

  format.fmt.pix.width = width;
  format.fmt.pix.height = height;

  // Actually set the video mode
  if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
    NODELET_ERROR("VIDIOC_S_FMT");
    NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
    return false;
  }

  return true;
}


bool FlirRos::request_buffers(int fd) {
  struct v4l2_requestbuffers req;
  CLEAR(req);
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  req.count = 1;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
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

  if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0) {
    NODELET_ERROR("VIDIOC_QUERYBUF");
    NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
    return false;
  }

  buffer = mmap(NULL, /* start anywhere */
                query_buffer.length, PROT_READ | PROT_WRITE, /* required */
                MAP_SHARED,                                  /* recommended */
                fd, query_buffer.m.offset);

  if (buffer == MAP_FAILED) {
    NODELET_ERROR("MAP_FAILED");
    NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
    return false;
  }

  memset(buffer, 0, query_buffer.length);
  NODELET_INFO("Allocated buffer of size %d KB", query_buffer.length / 1024);
  return true;
}

bool FlirRos::start_streaming(int fd) {
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    NODELET_ERROR("VIDIOC_STREAMON");
    NODELET_ERROR_STREAM("Error: " << std::string(strerror(errno)));
    return false;
  }

  return true;
}

void FlirRos::setup_ros_names() {
  camera_topic_name = camera_name + "/image";
  rect_topic_name = camera_name + "/image_rect";
  base_frame_id = camera_name + "/camera_link";
  img_opt_frame_id = camera_name + "/optical_frame";
}

void FlirRos::get_ros_param() {
  auto params =
      std::make_tuple(std::make_pair("device_name", std::ref(device_name)),
                      std::make_pair("camera_name", std::ref(camera_name)),
                      std::make_pair("intrinsic_url", std::ref(intrinsic_url)),
                      std::make_pair("width", std::ref(width)),
                      std::make_pair("height", std::ref(height)),
                      std::make_pair("send_every_n", std::ref(send_every_n)));

  auto load_param = [&](const auto& t) {
    CHECK_FATAL(!private_nh.hasParam(t.first),
                "Required param" << t.first << " not found!");
    private_nh.getParam(t.first, t.second);
    ROS_INFO_STREAM("Param " << t.first << ": " << t.second);
  };

  // https://stackoverflow.com/a/54053084
  std::apply([&](auto&&... args) { (load_param(args), ...); }, params);
}

void FlirRos::setup_ros() {
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
}

sensor_msgs::ImagePtr FlirRos::rectify_image(
    const sensor_msgs::ImageConstPtr& image_msg,
    const sensor_msgs::CameraInfoConstPtr& cam_info_ptr) {
  // update camera info
  cam_model.fromCameraInfo(cam_info_ptr);
  // get the image to cv format
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect_image;

  // rectify the image
  try {
    cam_model.rectifyImage(image, rect_image, CV_INTER_LINEAR);
  } catch (const image_geometry::Exception& e) {
    ROS_WARN_STREAM("Exception when rectifying: " << e.what());
    return nullptr;
  }

  // publish the rectified image
  sensor_msgs::ImagePtr rect_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
          .toImageMsg();
  return rect_msg;
}

void FlirRos::publish_frame(uint32_t bytes_used, ros::Time time) {
  sensor_msgs::ImagePtr img(new sensor_msgs::Image);
  img->width = width;
  img->height = height;
  img->is_bigendian = false;  // hopefully
  img->header.stamp = time;
  img->header.frame_id = img_opt_frame_id;

  // get current camera info data and set header
  sensor_msgs::CameraInfoPtr cam_info_ptr(
      new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
  cam_info_ptr->header.frame_id = img_opt_frame_id;
  cam_info_ptr->header.stamp = img->header.stamp;

  if (raw) {                // Can publish image directly, don't need opencv.
    img->step = width * 2;  // 2 bytes per pixel
    img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    img->data.insert(img->data.begin(), reinterpret_cast<char*>(buffer),
                     reinterpret_cast<char*>(buffer) + bytes_used);

  } else {  // Use opencv to convert from YUV420 to RGB
    // There might be a way to get rid of the initialization done inside
    // resize() if we're clever, but probably not necessary.
    auto bytes_per_pixel = 3;
    img->data.resize(width * height * bytes_per_pixel);

    auto luma_height = height + height / 2;  // 3/2 * height
    auto luma_width = width;

    // Directly use the input and output buffers with Mat as a wrapper,
    // so that we don't have any unnecessary copies.
    cv::Mat luma(luma_height, luma_width, CV_8UC1, buffer);
    cv::Mat rgb(height, width, CV_8UC3, &img->data[0]);

    // TODO(vasua): Make this conversion function run on GPU.
    // This function, when running at 60 hz on 2 cameras, takes upwards of 100%
    // CPU on the Xavier. Without this function (assuming the compiler isn't
    // being _really_ smart, CPU usage is basically 0. This should be a
    // relatively cheap operation to do on GPU as it's embarassingly parallel,
    // and GPUs are considerably better at float operations.
    timer.tic("cvtcolor");
    cv::cvtColor(luma, rgb, cv::COLOR_YUV2RGB_I420);
    timer.toc("cvtcolor");

    img->step = width * bytes_per_pixel;
    img->encoding = sensor_msgs::image_encodings::RGB8;
  }

  // Publish normal and rectified images if subscribed
  object_detection::publish_if_subscribed(image_pub, img, cam_info_ptr);
  if (rect_image_pub.getNumSubscribers() > 0) {
    timer.tic("rectify");
    // TODO(vasua): Make rectification faster too?
    // Rectification also isn't cheap, but this might be a harder one to do on
    // the GPU. Maybe we can log the non-rectified images at full frame rate,
    // and only rectify images that get used elsewhere?
    const auto rect_msg = rectify_image(img, cam_info_ptr);
    if (rect_msg != nullptr) {
      object_detection::publish_if_subscribed(rect_image_pub, rect_msg);
    }
    timer.toc("rectify");
  }
}

void FlirRos::publish_transform(const ros::Time& time, const tf::Vector3& trans,
                                const tf::Quaternion& q,
                                const std::string& from,
                                const std::string& to) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = from;
  msg.child_frame_id = to;

  tf::vector3TFToMsg(trans, msg.transform.translation);
  tf::quaternionTFToMsg(q, msg.transform.rotation);
  br.sendTransform(msg);
}

void FlirRos::publish_transforms(const ros::Time& time) {
  tf::Quaternion quaternion_optical;
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  const tf::Vector3 zero_translation{0, 0, 0};

  // Set camera link as Z upward, X to front and Y to left
  // Set optical frame as Z forward, X to right and Y downward
  publish_transform(time, zero_translation, quaternion_optical, base_frame_id,
                    img_opt_frame_id);
}

FlirRos::~FlirRos() {
  NODELET_INFO("Destructor called");

  if (stream) {
    stream = false;
    stream_thread.join();

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    CHECK_FATAL(ioctl(fd, VIDIOC_STREAMOFF, &type) < 0,
                "Couldn't properly close the stream!");
  }

  // Do we need to unmap the memory?
  // if (buffer != nullptr) {
  //     // Not checking error codes. Doesn't matter if it fails ...
  //     munmap(buffer, buffer_length);
  // }
}
}  // namespace flir_ros
