cmake_minimum_required(VERSION 3.0.2)

project(object_detector)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)


set(OpenCV_DIR "/home/jqzlca/software/opencv_install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

set(Eigen3_DIR "/home/jqzlca/software/eigen_install/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)

find_package(Threads REQUIRED)

include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


set(src
    ArucoTagDetect.cpp
    src/camera.cpp
    src/objectDetectorViaAruco.cpp
    src/objectDetectorOnRos.cpp)

catkin_package(     #此命令指定了生成的可执行性程序路径，到/dev/lib
  CATKIN_DEPENDS
  roscpp
  std_msgs
 )

add_executable(${PROJECT_NAME} ${src})

target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBS}
yaml-cpp
Threads::Threads)


