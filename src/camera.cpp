#include"camera.h"

camera::camera(int id){
    cap.open(id,cv::CAP_ANY);
      cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, 960);
      cap.set(cv::CAP_PROP_FPS, 30);
      cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
}
camera::~camera(){
    cap.release();
}

bool camera::imageRetrive(cv::Mat& FrameL,cv::Mat& FrameR)
{

    if (!cap.isOpened())
        return -1;

    key = cv::waitKey(1);
    cap >> frame;

    if (HNY_CV_002 == 0)  //
    {
        frame_L = frame(cv::Rect(0, 0, Image_Width, Image_height));
        frame_R = frame(cv::Rect(Image_Width, 0, Image_Width, Image_height));
    }
    else                  //
    {
        frame_L = frame(cv::Rect(Image_Width, 0, Image_Width, Image_height));
        frame_R = frame(cv::Rect(0, 0, Image_Width, Image_height));
    }

//    cv::namedWindow("Video_L", 1);
//    imshow("Video_L", frame_L);
    FrameL=frame_L.clone();
//    cv::namedWindow("Video_R", 1);
//    imshow("Video_R", frame_R);
    FrameR=frame_R.clone();




    if (key == 's')    //s save
    {
        sprintf(file_name, "/home/jqzlca/workspace/demo1/image//left/Left%u.jpg", num);
        imwrite(file_name, frame_L);
        sprintf(file_name, "/home/jqzlca/workspace/demo1/image/right/Right%u.jpg", num);
        imwrite(file_name, frame_R);
        num++;
        key = 0;
    }
    if(key=='q')// q quit
        return 0;

    return -1;

}
