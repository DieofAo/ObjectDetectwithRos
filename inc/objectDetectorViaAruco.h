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
const unsigned int NumFrameForMean=8;
const unsigned int MiniumNumFrameForMean=NumFrameForMean/2;
const float PrintedArucoSize=0.029;
//const unsigned int objectNum=3;

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
    Eigen::Quaterniond outputObjectPose;
};

struct object{
    unsigned int objectId;
    objectPose outputObject;
    float shape_size_obj[3];
    float shape_type_obj[3];//sphere, cylinder,cuboid
    Eigen::Isometry3d posture;
};

struct msgPose{
    struct objectPose pose;
    unsigned int objectId;
    int cameraId;
};

class arucoPose{
public:
    cv::Mat FramefromCameraL,FramefromCameraR;
    cv::Mat *resultFrame;
    std::vector<struct msgPose> *_outputObjectInformation;

    arucoPose(std::string& configYaml,std::vector<struct msgPose>& outputObjectInformation);
    ~arucoPose();
    void runDetectArucoTagPosByStereoCamera(cv::Mat& FrameDetectL,cv::Mat& FrameDetectR,int cameraId);
void configCameraInformation(int cameraId);
private:
    std::vector<std::vector<Tag>> TagL,TagR;
    std::vector<std::vector<Tag*>> DimensionBasedMarkIdTagL,DimensionBasedMarkIdTagR;
    std::vector<object> output;
    unsigned int countofTag=0;
    cv::Mat intrinsic_matrixL,distortion_matrixL,intrinsic_matrixR,distortion_matrixR,transMatrixFromR2L;
    cv::Mat projMatrixL,projMatrixR;

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
void configYamlParams(const YAML::Node& ParamsYaml,float (&ConfigMatrix)[3]);
#endif
