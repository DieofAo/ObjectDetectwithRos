#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>

class camera{
    public:
    camera(int id);
    ~camera();
    bool imageRetrive(cv::Mat& FrameL,cv::Mat& FrameR);
private:
    uchar HNY_CV_002 = 0;

    const unsigned int Image_Width=1280;
    const unsigned int Image_height=720;


    cv::VideoCapture cap;
    cv::Mat frame,frame_L,frame_R;
    char file_name[100];
    uchar num = 1;
    int key = 0;

};
