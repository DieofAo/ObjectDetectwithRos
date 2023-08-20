#include<iostream>
#include"camera.h"
//#include"arucoDetect.h"
#include"objectDetectorViaAruco.h"

cv::Mat leftFrame,rightFrame;



int main(){

    arucoPose PositionDetect;
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

