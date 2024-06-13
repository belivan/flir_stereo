#ifndef OBJECT_DETECTION_COMMON_SERIAL_H
#define OBJECT_DETECTION_COMMON_SERIAL_H

#include <ros/serialization.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>

namespace object_detection {

template <typename T>
size_t write_msg_to_file(const std::string& path, const T& msg) {
  boost::filesystem::path boost_path(path);
  boost::filesystem::create_directories(boost_path.parent_path());

  std::ofstream out(path, std::ios::binary);
  auto serialized_msg = ros::serialization::serializeMessage(msg);
  out.write(reinterpret_cast<char*>(serialized_msg.message_start),
            serialized_msg.num_bytes);

  return serialized_msg.num_bytes;
}

template <typename T>
T load_msg_from_file(const std::string& path) {
  auto size = boost::filesystem::file_size(path);

  // http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
  boost::shared_array<uint8_t> buf(new uint8_t[size]);

  {
    std::ifstream in(path, std::ios::binary);
    in.read(reinterpret_cast<char*>(buf.get()), size);
  }

  T msg;
  ros::serialization::IStream istream(buf.get(), size);
  ros::serialization::Serializer<T>::read(istream, msg);

  return msg;
}
}  // namespace object_detection

#endif  // OBJECT_DETECTION_COMMON_SERIAL_H
