#include"camera.h"
#include"objectDetectorViaAruco.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include "object_detector/objectPose.h"


class objectDetectorOnRos{
public:
    int cameraId;
    arucoPose *PositionDetect;
    camera *steroCamera;
    camera *steroCamera2;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Publisher *pubGlobalLRaw,*pubGlobalRRaw,*pubGlobalResult;
    image_transport::Publisher *pubHandLRaw,*pubHandRRaw,*pubHandResult;
    ros::Publisher *objectPosture;
    object_detector::objectPose *msg;
    std::vector<struct msgPose> outputObjectInformation;

    std::string config_Yaml;

    cv::Mat originalLeftFrame,originalRightFrame,ArucoDetectedLeftFrame;


    objectDetectorOnRos(ros::NodeHandle& nh,int Id);
    ~objectDetectorOnRos();
    void run(int cameraId);
    void rosImageView(cv::Mat& imageLRaw,cv::Mat& imageRRaw,cv::Mat& imageResult,int cameraID);
};
