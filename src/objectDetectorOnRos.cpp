#include"objectDetectorOnRos.h"
objectDetectorOnRos::~objectDetectorOnRos(){

}

objectDetectorOnRos::objectDetectorOnRos(ros::NodeHandle& nh,int Id):_nh(),_it(_nh){
    _nh=nh;
    cameraId=Id;
    config_Yaml="";
    _nh.getParam("config_Yaml",config_Yaml);

    PositionDetect=new arucoPose(config_Yaml);
    steroCamera=new camera(cameraId);

    pubLRaw=new image_transport::Publisher(_it.advertise("LRaw", 5));
    pubRRaw=new image_transport::Publisher(_it.advertise("RRaw", 5));
    pubResult=new image_transport::Publisher(_it.advertise("Result", 5));


}
void objectDetectorOnRos::run(){
    cv::Mat leftFrame,rightFrame;


    steroCamera->imageRetrive(leftFrame,rightFrame);

    PositionDetect->runDetectArucoTagPosByStereoCamera(leftFrame,rightFrame);

    ArucoDetectedLeftFrame=PositionDetect->FramefromCameraL;
    rosImageView(leftFrame,rightFrame,ArucoDetectedLeftFrame);

}


void objectDetectorOnRos::rosImageView(cv::Mat& imageLRaw,
                                       cv::Mat& imageRRaw,
                                       cv::Mat& imageResult){
    sensor_msgs::ImagePtr msgLRaw,msgRRaw,msgResult;

    std_msgs::Header header;
    header.frame_id = ros::names::resolve(ros::this_node::getNamespace() ,"global") + "/camera_link";
    //capture the image and publish it
    if (_nh.ok()) {
        ros::Time time = ros::Time::now();
        header.stamp = time;             //图像里头的时间戳
        //convert the mat object to the ros image msg with the header
        msgLRaw = cv_bridge::CvImage(header, "bgr8", imageLRaw).toImageMsg();
        msgRRaw = cv_bridge::CvImage(header, "bgr8", imageRRaw).toImageMsg();
        msgResult = cv_bridge::CvImage(header, "bgr8", imageResult).toImageMsg();

        pubLRaw->publish(msgLRaw);
        pubRRaw->publish(msgRRaw);
        if(!ArucoDetectedLeftFrame.empty())
            pubResult->publish(msgResult);

    }
}
