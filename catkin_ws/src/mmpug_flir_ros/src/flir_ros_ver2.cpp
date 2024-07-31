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

    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();
    get_ros_param();

    timer.set_name(getName());
    timer.tic("init_camera");

    fd.reset(open(device_name.c_str(), O_RDWR));
    CHECK_FATAL(fd < 0, "Unable to open device " << device_name);

    struct v4l2_capability cap;
    CLEAR(cap);
    CHECK_FATAL(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0,
                "Unable to query capabilities!");
    CHECK_FATAL(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE),
                "Device can't stream video!");

    CHECK_FATAL(!set_format(fd, raw), "Unable to set video format!");

    CHECK_FATAL(!request_buffers(fd), "Requesting buffers failed!");

    CHECK_FATAL(!start_streaming(fd), "Stream start failed!");

    NODELET_INFO("Ready to start streaming camera!");
    stream = true;

    timer.toc("init_camera");

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
                break;
            }
            timer.toc("queue");

            timer.tic("dequeue");
            if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
                NODELET_INFO("Couldn't dequeue :(");
                NODELET_INFO_STREAM("Error: " << std::string(strerror(errno)));
                break;
            }
            timer.toc("dequeue");

            if (++count != send_every_n) {
                continue;
            }
            count = 0;

            timer.tic("publish_frame");
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

    if (raw) {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;  // YU12
    } else {
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;  // NV12
    }

    format.fmt.pix.width = width;
    format.fmt.pix.height = height;

    NODELET_INFO_STREAM("Trying to set format: " << format.fmt.pix.pixelformat
                        << " width: " << format.fmt.pix.width
                        << " height: " << format.fmt.pix.height);

    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        NODELET_ERROR("VIDIOC_S_FMT failed with error: " << strerror(errno));
        return false;
    }

    struct v4l2_format verify_format;
    CLEAR(verify_format);
    verify_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &verify_format) < 0) {
        NODELET_ERROR("VIDIOC_G_FMT failed with error: " << strerror(errno));
        return false;
    }

    NODELET_INFO_STREAM("Video format set to: "
                        << " width: " << verify_format.fmt.pix.width
                        << " height: " << verify_format.fmt.pix.height
                        << " pixelformat: " << verify_format.fmt.pix.pixelformat);

    return true;
}

bool FlirRos::request_buffers(int fd) {
    struct v4l2_requestbuffers req;
    CLEAR(req);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    req.count = 1;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        NODELET_ERROR("VIDIOC_REQBUFS failed with error: " << strerror(errno));
        return false;
    }

    NODELET_INFO_STREAM("Device requested " << req.count << " buffers!");

    struct v4l2_buffer query_buffer;
    CLEAR(query_buffer);
    query_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    query_buffer.memory = V4L2_MEMORY_MMAP;
    query_buffer.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &query_buffer) < 0) {
        NODELET_ERROR("VIDIOC_QUERYBUF failed with error: " << strerror(errno));
        return false;
    }

    buffer = mmap(NULL, query_buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, query_buffer.m.offset);

    if (buffer == MAP_FAILED) {
        NODELET_ERROR("mmap failed with error: " << strerror(errno));
        return false;
    }

    memset(buffer, 0, query_buffer.length);
    NODELET_INFO("Allocated buffer of size %d KB", query_buffer.length / 1024);
    return true;
}

bool FlirRos::start_streaming(int fd) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        NODELET_ERROR("VIDIOC_STREAMON failed with error: " << strerror(errno));
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
    auto params = std::make_tuple(
        std::make_pair("device_name", std::ref(device_name)),
        std::make_pair("camera_name", std::ref(camera_name)),
        std::make_pair("intrinsic_url", std::ref(intrinsic_url)),
        std::make_pair("width", std::ref(width)),
        std::make_pair("height", std::ref(height)),
        std::make_pair("send_every_n", std::ref(send_every_n)),
        std::make_pair("raw", std::ref(raw))
    );

    auto load_param = [&](const auto& t) {
        CHECK_FATAL(!private_nh.hasParam(t.first),
                    "Required param " << t.first << " not found!");
        private_nh.getParam(t.first, t.second);
        ROS_INFO_STREAM("Param " << t.first << ": " << t.second);
    };

    std::apply([&](auto&&... args) { (load_param(args), ...); }, params);
}

void FlirRos::setup_ros() {
    setup_ros_names();

    it.reset(new image_transport::ImageTransport(nh));
    cinfo.reset(new camera_info_manager::CameraInfoManager(nh));

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
    cam_model.fromCameraInfo(cam_info_ptr);
    const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat rect_image;

    try {
        cam_model.rectifyImage(image, rect_image, CV_INTER_LINEAR);
    } catch (const image_geometry::Exception& e) {
        ROS_WARN_STREAM("Exception when rectifying: " << e.what());
        return nullptr;
    }

    sensor_msgs::ImagePtr rect_msg =
        cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect_image)
            .toImageMsg();
    return rect_msg;
}

void FlirRos::publish_frame(uint32_t bytes_used, ros::Time time) {
    sensor_msgs::ImagePtr img(new sensor_msgs::Image);
    img->width = width;
    img->height = height;
    img->is_bigendian = false;
    img->header.stamp = time;
    img->header.frame_id = img_opt_frame_id;

    sensor_msgs::CameraInfoPtr cam_info_ptr(
        new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
    cam_info_ptr->header.frame_id = img_opt_frame_id;
    cam_info_ptr->header.stamp = img->header.stamp;

    if (raw) {
        img->step = width * 2;
        img->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        img->data.insert(img->data.begin(), reinterpret_cast<char*>(buffer),
                         reinterpret_cast<char*>(buffer) + bytes_used);
    } else {
        auto bytes_per_pixel = 3;
        img->data.resize(width * height * bytes_per_pixel);

        auto luma_height = height + height / 2;
        auto luma_width = width;

        cv::Mat luma(luma_height, luma_width, CV_8UC1, buffer);
        cv::Mat rgb(height, width, CV_8UC3, &img->data[0]);

        timer.tic("cvtcolor");
        cv::cvtColor(luma, rgb, cv::COLOR_YUV2RGB_I420);
        timer.toc("cvtcolor");

        img->step = width * bytes_per_pixel;
        img->encoding = sensor_msgs::image_encodings::RGB8;
    }

    object_detection::publish_if_subscribed(image_pub, img, cam_info_ptr);
    if (rect_image_pub.getNumSubscribers() > 0) {
        timer.tic("rectify");
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
}
}  // namespace flir_ros
