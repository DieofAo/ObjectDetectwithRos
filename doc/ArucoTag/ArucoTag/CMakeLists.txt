cmake_minimum_required(VERSION 3.5)

project(ArucoTag LANGUAGES CXX)




set(OpenCV_DIR "/home/jqzlca/software/opencv_install/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)

set(Eigen3_DIR "/home/jqzlca/software/eigen_install/share/eigen3/cmake")
find_package(Eigen3 REQUIRED)








add_executable(ArucoTag main.cpp)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)
