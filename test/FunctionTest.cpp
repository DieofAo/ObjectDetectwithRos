#include<iostream>
#include"camera.h"
//#include"arucoDetect.h"
#include"objectDetectorViaAruco.h"

cv::Mat leftFrame,rightFrame;



int main(){
std::string path="/home/jqzlca/workspace/catkin_ws/src/jqz_ws/object_detector/config/object.yaml";
std::vector<struct msgPose> outputObjectInformation;
    arucoPose PositionDetect(path,outputObjectInformation);
    camera steroCamera(0);

    while (1) {
//        std::thread global=std::thread([&](){
            steroCamera.imageRetrive(leftFrame,rightFrame);
            PositionDetect.runDetectArucoTagPosByStereoCamera(leftFrame,rightFrame);

//        });
        //        std::thread hand=std::thread([&](){
        //            hand_detector.run();
        //        });

//        global.join();
        //        hand.join();

    }
    return 0;
}

