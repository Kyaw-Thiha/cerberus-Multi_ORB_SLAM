cmake_minimum_required(VERSION 3.8)
project(multicam_slam_ros2_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0...<4 REQUIRED)
# Eigen >= 3.4's CMake package only exports the Eigen3::Eigen imported
# target and no longer sets the legacy EIGEN3_INCLUDE_DIR variable used
# by the core library's own headers. Recover it from the target, mirroring
# the workaround in the top-level CMakeLists.txt.
if(NOT EIGEN3_INCLUDE_DIR AND TARGET Eigen3::Eigen)
  get_target_property(EIGEN3_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
endif()

# ORB_SLAM2 (this fork's core library) is an already-built, plain (non-ROS)
# shared library that lives one directory up
# (third_party/slam/multicam_slam/lib/libORB_SLAM2.so), built by the
# top-level CMakeLists.txt in that repo. This package only links against
# it -- it does not rebuild the core library itself.
set(MULTICAM_SLAM_ROOT ${CMAKE_SOURCE_DIR}/..)

find_library(ORB_SLAM2_LIBRARY
  NAMES ORB_SLAM2
  PATHS ${MULTICAM_SLAM_ROOT}/lib
  NO_DEFAULT_PATH
)
if(NOT ORB_SLAM2_LIBRARY)
  message(FATAL_ERROR
    "libORB_SLAM2.so not found in ${MULTICAM_SLAM_ROOT}/lib -- build the core "
    "ORB_SLAM2 library first (see the top-level CMakeLists.txt / build.sh).")
endif()

find_library(DBoW2_LIBRARY
  NAMES DBoW2
  PATHS ${MULTICAM_SLAM_ROOT}/Thirdparty/DBoW2/lib
  NO_DEFAULT_PATH
)
find_library(g2o_LIBRARY
  NAMES g2o
  PATHS ${MULTICAM_SLAM_ROOT}/Thirdparty/g2o/lib
  NO_DEFAULT_PATH
)
if(NOT DBoW2_LIBRARY OR NOT g2o_LIBRARY)
  message(FATAL_ERROR
    "libDBoW2.so / libg2o.so not found under ${MULTICAM_SLAM_ROOT}/Thirdparty -- "
    "build the core ORB_SLAM2 library first.")
endif()

include_directories(
  ${MULTICAM_SLAM_ROOT}
  ${MULTICAM_SLAM_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(multicam_slam_node src/multicam_slam_node.cpp)

target_include_directories(multicam_slam_node PUBLIC
  ${MULTICAM_SLAM_ROOT}
  ${MULTICAM_SLAM_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
)

# The system OpenCV package pulls in libopencv_viz -> VTK, and on this
# distro one VTK module (libvtkIOExport) has an unresolved libfmt symbol of
# its own. We never use viz/VTK functionality (this fork's core library is
# built without Pangolin, per the top-level CMakeLists.txt change), so
# allow the final executable link to proceed without fully resolving
# symbols in that indirectly-linked, unused shared library.
target_link_options(multicam_slam_node PRIVATE -Wl,--allow-shlib-undefined)

target_link_libraries(multicam_slam_node
  ${ORB_SLAM2_LIBRARY}
  ${DBoW2_LIBRARY}
  ${g2o_LIBRARY}
  ${OpenCV_LIBS}
)

ament_target_dependencies(multicam_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  message_filters
)

install(TARGETS multicam_slam_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
