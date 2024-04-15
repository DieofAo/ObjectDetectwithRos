#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main(int argc, char** argv)
{
     // Initialize ROS

    int width=1400;
    int height=1980;
    int square=200;
    int step=240;
    //nh.getParam("width1",width);
    //nh.getParam("height1",height);
    //nh.getParam("square1",square);
    //nh.getParam("step1",step);
    //shared_ptr<cv::Mat> whiteBackGround(new cv::Mat(2235,1580,CV_8UC1,cv::Scalar(255,255,255)));
    cv::Mat* whiteBackGround = new cv::Mat(height,width,CV_8UC1,cv::Scalar(255,255,255));
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    int row=0;
    int col=0;
    for(int i=0;i<48;i++){
        cv::aruco::drawMarker(dictionary, i, square, markerImage, 1);
        cv::Rect roi_rect = cv::Rect(row, col, markerImage.cols, markerImage.rows);
        markerImage.copyTo((*whiteBackGround)(roi_rect));
        row+=step;
        if(row>=width){
            col+=step;
            row=0;
        }
    }
    cv::imshow("result", *whiteBackGround);
    cv::imwrite("./aruco_tag.png",*whiteBackGround);
    cv::imshow("test", markerImage);//显示marker
    cv::waitKey();
   // sleep(1000);
    return 0;
}
