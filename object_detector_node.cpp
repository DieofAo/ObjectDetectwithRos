#include"objectDetectorOnRos.h"


int main(int argc, char** argv){

    ros::init(argc, argv, "objectDetectImage_node");
    ros::NodeHandle nh("~");

    int globalCameraId,handCameraId;
    nh.getParam("gloabalCameraId",globalCameraId);
    nh.getParam("handCameraId",handCameraId);

//    objectDetectorOnRos hand_detector(nh,handCameraId);
    objectDetectorOnRos global_detector(nh,globalCameraId);

    ros::Rate looprate(30);

    while (1) {
        std::thread global=std::thread([&](){
            global_detector.run();
        });
//        std::thread hand=std::thread([&](){
//            hand_detector.run();
//        });
        global.join();
//        hand.join();
        looprate.sleep();

    }
    return 0;
}

