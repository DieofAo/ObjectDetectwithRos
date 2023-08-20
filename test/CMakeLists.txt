cmake_minimum_required(VERSION 3.0.2)

project(object_detector_test)


set(OpenCV_DIR "/home/jqzlca/software/opencv_install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

set(Eigen3_DIR "/home/jqzlca/software/eigen_install/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)

find_package(Threads REQUIRED)

include_directories(
  ../inc
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


set(src
    FunctionTest.cpp
    ../src/camera.cpp
    ../src/objectDetectorViaAruco.cpp
    )

add_executable(${PROJECT_NAME} ${src})

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
yaml-cpp
Threads::Threads)


