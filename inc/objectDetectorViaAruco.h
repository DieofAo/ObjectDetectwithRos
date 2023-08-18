#ifndef ARUCO_TAG_POSITION_DETECTION_H
#define ARUCO_TAG_POSITION_DETECTION_H

#include <Eigen/Eigen>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <thread>


const unsigned int ArucoTagCount=48;
const unsigned int NumFrameForMean=10;
const unsigned int MiniumNumFrameForMean=NumFrameForMean/2;
const float PrintedArucoSize=0.029;
const unsigned int objectNum=3;

struct Tag{
    unsigned int markId;
    std::vector<cv::Point2f> markCorners;

    cv::Vec3d rotVec;
    cv::Vec3d tvecs;
    cv::Point2f centerPoint;
    cv::Point3d reconstructCenterPoint;//only in TgaL
    std::vector<cv::Point3d> reconstructMarkCorners;//only in TgaL
    int frame;
    Eigen::Isometry3d PostureforQuatenionVector;
    Eigen::Quaterniond quaternionVector;
    Eigen::Quaterniond averagequaternion;

};

struct objectPose{
    Tag objectTag;
    unsigned int objectMarkId;
    Eigen::Isometry3d objectPosture;
    Eigen::Isometry3d tranformationFromTag2Object;
};

struct object{
    unsigned int objectId;
    objectPose outputObject;
    float shape_size_obj[3];
    float shape_type_obj[3];//sphere, cylinder,cuboid
    Eigen::Isometry3d posture;
};

class arucoPose{
public:
    arucoPose();
    ~arucoPose();
    void runDetectArucoTagPosByStereoCamera(cv::Mat& FrameDetectL,cv::Mat& FrameDetectR,cv::Mat& ResultFrame);

private:
    std::vector<std::vector<Tag>> TagL,TagR;
    std::vector<std::vector<Tag*>> DimensionBasedMarkIdTagL,DimensionBasedMarkIdTagR;
    std::vector<object> output;
    unsigned int countofTag=0;
    cv::Mat intrinsic_matrixL,distortion_matrixL,intrinsic_matrixR,distortion_matrixR,transMatrixFromR2L;
    cv::Mat FramefromCameraL,FramefromCameraR;
    YAML::Node ObjectForDetecting;



    void detectArucoTagPosition(cv::Mat& FrameDetect,cv::Mat& intrinsic_matrix,cv::Mat& distortion_matrix,std::vector<Tag>& rotVec);
    void matchArucoTagFromSingalFrame(std::vector<Tag>& RotVecofTagL,std::vector<Tag>& RotVecofTagR);
    void TriangulationRangingMethod();
    void crossCenterPoint(Tag& TagForCrossCenter);
    void TransferFrameBasedDimension2MarkId();
    void ProcessDimensionBasedMarkId();
    void chooseStableArucoTag(std::vector<Tag*>& ForChooseStableArucoTag);//choose the unstable ArucoTag
    void calculateNewCoordinatebyReconstructCorners(Tag* ForCalculateNewCoordinate,Eigen::Matrix3d& NewRotation);
    float poseDiff_fromFrame1ToFrame2InBase(Eigen::Isometry3d& _pose1InBase,Eigen::Isometry3d& _pose2InBase);

    void AverageQuatertionfromMatchedMultiflame(std::vector<Tag*> BasedIdMarkTag,Eigen::Vector4d& averageResult);

    void outputArucoPosture();

};
#endif
