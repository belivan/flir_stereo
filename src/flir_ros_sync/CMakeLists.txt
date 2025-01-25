cmake_minimum_required(VERSION 3.8)
project(flir_ros_sync)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED) 

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Define the Boson SDK source directory
set(BOSON_SDK_CLIENT_FILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/ClientFiles_C)
set(BOSON_SDK_FSLP_FILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src)
set(BOSON_SDK_FSLP_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src/inc)
set(BOSON_SDK_FSLP_LINUX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/script/Boson_SDK/FSLP_Files/src/linux)

# Add SDK source files
# set(BOSON_SDK_SRCS
#   ${BOSON_SDK_SRC_DIR}/Client_API.c
#   ${BOSON_SDK_SRC_DIR}/EnumTypes.h
#   ${BOSON_SDK_SRC_DIR}/ReturnCodes.h
#   ${BOSON_SDK_SRC_DIR}/Client_Packager.c
# )
file(GLOB BOSON_SDK_SRCS
  ${BOSON_SDK_CLIENT_FILES_DIR}/*.c
  ${BOSON_SDK_FSLP_FILES_DIR}/*.c
  ${BOSON_SDK_FSLP_INC_DIR}/*.h
  ${BOSON_SDK_FSLP_LINUX_DIR}/*.c
  ${BOSON_SDK_FSLP_LINUX_DIR}/*.h
)

set_source_files_properties(${BOSON_SDK_SRCS} PROPERTIES LANGUAGE C)

# Create a static library for BOSON_SDK
add_library(Boson_SDK STATIC
  ${BOSON_SDK_SRCS}
)

# Include directories for BOSON_SDK
target_include_directories(Boson_SDK PUBLIC
  ${BOSON_SDK_CLIENT_FILES_DIR}
  ${BOSON_SDK_FSLP_INC_DIR}
  ${BOSON_SDK_FSLP_LINUX_DIR}
)

# Library
add_library(${PROJECT_NAME} SHARED
  include/flir_ros_sync/flir_ros_sync.h
  include/flir_ros_sync/fd_guard.h
  include/flir_ros_sync/image_transport.h
  src/flir_ros_sync.cpp
  src/rawBoson.cpp
  src/bytes.cpp
  src/serial.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${BOSON_SDK_CLIENT_FILES_DIR}
  ${BOSON_SDK_FSLP_INC_DIR}
  ${BOSON_SDK_FSLP_LINUX_DIR}
)

# Link against libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Boson_SDK
  # ${BOSON_SDK_SRC_DIR}/FSLP_32.so
  # ${BOSON_SDK_SRC_DIR}/FSLP_64.so
  pthread
)

# Register component
rclcpp_components_register_nodes(${PROJECT_NAME} "flir_ros_sync::FlirRos")

# Add executable
add_executable(set_sync_mode 
  src/set_sync_mode.cpp
  src/rawBoson.cpp
  src/bytes.cpp
  src/serial.cpp
)

# Dependencies for the executable
ament_target_dependencies(set_sync_mode
  rclcpp
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  image_geometry
  OpenCV
  std_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executable
install(
  TARGETS set_sync_mode
  DESTINATION lib/${PROJECT_NAME}
)

# Install py scripts
install(PROGRAMS
  script/trigger_ffc.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  script/check_teensy_status.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Boson SDK module
install(
  DIRECTORY script/Boson_SDK
  DESTINATION lib/${PROJECT_NAME}
)

# Install data
install(DIRECTORY data/
    DESTINATION share/${PROJECT_NAME}/data
)

# Install config
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files if any
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()