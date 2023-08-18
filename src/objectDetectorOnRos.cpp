#include"objectDetectorOnRos.h"
objectDetectorOnRos::~objectDetectorOnRos(){

}

objectDetectorOnRos::objectDetectorOnRos(ros::NodeHandle& nh,int Id):_nh(),_it(_nh){
    cameraId=Id;
    _nh=nh;
    pubLRaw=new image_transport::Publisher(_it.advertise("LRaw", 5));
    pubRRaw=new image_transport::Publisher(_it.advertise("RRaw", 5));
    pubResult=new image_transport::Publisher(_it.advertise("Result", 5));


}
void objectDetectorOnRos::run(){
    cv::Mat leftFrame,rightFrame;
    camera steroCamera(cameraId);
    while (1) {
        if(steroCamera.imageRetrive(leftFrame,rightFrame)==0)
            break;
        PositionDetect.runDetectArucoTagPosByStereoCamera(leftFrame,rightFrame,ArucoDetectedLeftFrame);
        rosImageView(leftFrame,rightFrame,ArucoDetectedLeftFrame);

    }
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
        msgLRaw = cv_bridge::CvImage(header, "bgr8", imageLRaw).toImageMsg();   //生成图像消息
        msgRRaw = cv_bridge::CvImage(header, "bgr8", imageRRaw).toImageMsg();   //生成图像消息

        msgResult = cv_bridge::CvImage(header, "bgr8", imageResult).toImageMsg();   //生成图像消息

        pubLRaw->publish(msgLRaw);
        pubRRaw->publish(msgRRaw);
        if(!ArucoDetectedLeftFrame.empty())
            pubResult->publish(msgResult);

    }
}
