#include"objectDetectorOnRos.h"
objectDetectorOnRos::~objectDetectorOnRos(){

}

objectDetectorOnRos::objectDetectorOnRos(ros::NodeHandle& nh,int Id):_nh(),_it(_nh){
    _nh=nh;
    cameraId=Id;
    config_Yaml="";
    _nh.getParam("config_Yaml",config_Yaml);

    PositionDetect=new arucoPose(config_Yaml,outputObjectInformation);
    steroCamera=new camera(cameraId);

    pubLRaw=new image_transport::Publisher(_it.advertise("LRaw", 5));
    pubRRaw=new image_transport::Publisher(_it.advertise("RRaw", 5));
    pubResult=new image_transport::Publisher(_it.advertise("Result", 5));
    objectPosture=new ros::Publisher(_nh.advertise<object_detector::objectPose>("object_info",10));
    msg=new object_detector::objectPose;

    //    outputObjectInformation=new struct object;
//    outputObjectInformation=(struct object*) malloc(sizeof(struct object));

//        objectPosture=new ros::Publisher;
//        *objectPosture=_nh.advertise<object_detector::objectPose>("/object_info",10);


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
//    ros::Rate rate(10);
    //capture the image and publish it
    if (_nh.ok()) {
        ros::Time time = ros::Time::now();
        header.stamp = time;             //图像里头的时间戳
        //convert the mat object to the ros image msg with the header
        msgLRaw = cv_bridge::CvImage(header, "bgr8", imageLRaw).toImageMsg();
        msgRRaw = cv_bridge::CvImage(header, "bgr8", imageRRaw).toImageMsg();
        msgResult = cv_bridge::CvImage(header, "bgr8", imageResult).toImageMsg();

        //config object pose for msg
        if(outputObjectInformation.size()){
            for(size_t i=0;i<outputObjectInformation.size();i++){
                msg->t1=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().x();
                msg->t2=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().y();
                msg->t3=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().z();
                msg->x=outputObjectInformation.at(i).pose.outputObjectPose.x();
                msg->y=outputObjectInformation.at(i).pose.outputObjectPose.y();
                msg->z=outputObjectInformation.at(i).pose.outputObjectPose.z();
                msg->w=outputObjectInformation.at(i).pose.outputObjectPose.w();
                msg->id=outputObjectInformation.at(i).objectId;
                objectPosture->publish(*msg);
                ROS_INFO("msg:",msg->id);
            }
        }

        pubLRaw->publish(msgLRaw);
        pubRRaw->publish(msgRRaw);
        if(!ArucoDetectedLeftFrame.empty())
            pubResult->publish(msgResult);
//        rate.sleep();

    }
}
