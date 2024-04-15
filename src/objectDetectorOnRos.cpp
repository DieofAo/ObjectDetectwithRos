#include"objectDetectorOnRos.h"
objectDetectorOnRos::~objectDetectorOnRos(){
    delete PositionDetect;
    delete steroCamera;
    delete pubGlobalLRaw;
    delete pubGlobalRRaw;
    delete pubGlobalResult;
    delete pubHandLRaw;
    delete pubHandRRaw;
    delete pubHandResult;
    delete objectPosture;
    delete msg;

}
//hand camera code 25-26;98-103

objectDetectorOnRos::objectDetectorOnRos(ros::NodeHandle& nh,int Id):_nh(),_it(_nh){
    _nh=nh;
    cameraId=Id;
    config_Yaml="";
    _nh.getParam("config_Yaml",config_Yaml);

    PositionDetect=new arucoPose(config_Yaml,outputObjectInformation);
    steroCamera=new camera(cameraId);
    if(cameraId==0)
        steroCamera2=new camera(4);

    pubGlobalLRaw=new image_transport::Publisher(_it.advertise("GlobalLRaw", 5));
    pubGlobalRRaw=new image_transport::Publisher(_it.advertise("GlobalRRaw", 5));
    pubGlobalResult=new image_transport::Publisher(_it.advertise("GlobalResult", 5));
    pubHandLRaw=new image_transport::Publisher(_it.advertise("HandLRaw", 5));
    pubHandRRaw=new image_transport::Publisher(_it.advertise("HandRRaw", 5));
    pubHandResult=new image_transport::Publisher(_it.advertise("HandResult", 5));
    objectPosture=new ros::Publisher(_nh.advertise<object_detector::objectPose>("object_info",10));
    msg=new object_detector::objectPose;

}

void objectDetectorOnRos::run(int cameraId){
    cv::Mat leftFrame,tempRightFrame,tempLeftFrame,rightFrame;

    if(cameraId==0){
        steroCamera->imageRetrive(leftFrame,tempRightFrame);
        steroCamera2->imageRetrive(tempLeftFrame,rightFrame);
    }
    else
        steroCamera->imageRetrive(leftFrame,rightFrame);


    PositionDetect->runDetectArucoTagPosByStereoCamera(leftFrame,rightFrame,cameraId);

    ArucoDetectedLeftFrame=PositionDetect->FramefromCameraL;
    rosImageView(leftFrame,rightFrame,ArucoDetectedLeftFrame,cameraId);

}


void objectDetectorOnRos::rosImageView(cv::Mat& imageLRaw,
                                       cv::Mat& imageRRaw,
                                       cv::Mat& imageResult,
                                       int cameraId){
    sensor_msgs::ImagePtr msgLRaw,msgRRaw,msgResult;

    std_msgs::Header header;
    header.frame_id = ros::names::resolve(ros::this_node::getNamespace() ,"global") + "/camera_link";
    ros::Rate rate(10);
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
                msg->t_x=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().x();
                msg->t_y=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().y();
                msg->t_z=outputObjectInformation.at(i).pose.tranformationFromTag2Object.translation().z();
                msg->x=outputObjectInformation.at(i).pose.outputObjectPose.x();
                msg->y=outputObjectInformation.at(i).pose.outputObjectPose.y();
                msg->z=outputObjectInformation.at(i).pose.outputObjectPose.z();
                msg->w=outputObjectInformation.at(i).pose.outputObjectPose.w();
                msg->objectId=outputObjectInformation.at(i).objectId;
                msg->cameraId=cameraId;

                objectPosture->publish(*msg);

            }
        }
        if(cameraId==0){
            pubGlobalLRaw->publish(msgLRaw);
            pubGlobalRRaw->publish(msgRRaw);
            if(!ArucoDetectedLeftFrame.empty())
                pubGlobalResult->publish(msgResult);
        }
//        if(cameraId==4){
//            pubHandLRaw->publish(msgLRaw);
//            pubHandRRaw->publish(msgRRaw);
//            if(!ArucoDetectedLeftFrame.empty())
//                pubHandResult->publish(msgResult);
//        }

        rate.sleep();

    }
}
