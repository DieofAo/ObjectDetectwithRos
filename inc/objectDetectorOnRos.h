#include"camera.h"
#include"objectDetectorViaAruco.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>





class objectDetectorOnRos{
public:
    int cameraId;
    arucoPose PositionDetect;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Publisher *pubLRaw,*pubRRaw,*pubResult;

    cv::Mat originalLeftFrame,originalRightFrame,ArucoDetectedLeftFrame,ObjectResult;


    objectDetectorOnRos(ros::NodeHandle& nh,int Id);
    ~objectDetectorOnRos();
    void run();
    void rosImageView(cv::Mat& imageLRaw,cv::Mat& imageRRaw,cv::Mat& imageResult);
};
