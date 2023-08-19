#include "objectDetectorViaAruco.h"


arucoPose::~arucoPose(){

}

arucoPose::arucoPose(){
    distortion_matrixL=(cv::Mat_<double>(5,1)<<0.0128,-0.0343,0.0023,-0.00011,0);
    intrinsic_matrixL=(cv::Mat_<double>(3,3)<<921.6992,0,653.3826,
                       0,918.4418,364.1299,
                       0,0,1);
    distortion_matrixR=(cv::Mat_<double>(5,1)<<0.0135,-0.0339,0.0032,-0.00011,0);
    intrinsic_matrixR=(cv::Mat_<double>(3,3)<<920.5909,0,646.5033,
                       0,917.4791,370.2785,
                       0,0,1);
    transMatrixFromR2L=(cv::Mat_<double>(4,4)<<1.0,0.0023,1.4284e-4,-65.2972,
                        -0.0023,1.0,-0.0066,0.0304,
                        1.5806e-4,0.0066,1.0,-0.1852,
                        0,0,0,1);

    TagL.resize(NumFrameForMean,std::vector<Tag>(ArucoTagCount));
    TagR.resize(NumFrameForMean,std::vector<Tag>(ArucoTagCount));
    DimensionBasedMarkIdTagL.resize(ArucoTagCount);
    DimensionBasedMarkIdTagR.resize(ArucoTagCount);
    output.resize(objectNum);
    ObjectForDetecting=YAML::LoadFile("/home/jqzlca/workspace/arucotags_target_pose/object.yaml");
}

void arucoPose::runDetectArucoTagPosByStereoCamera(cv::Mat& FrameDetectL,
                                                   cv::Mat& FrameDetectR){
    FramefromCameraL=FrameDetectL.clone();
    FramefromCameraR=FrameDetectR.clone();
    try {
        std::thread left=std::thread([&](){
            detectArucoTagPosition(FramefromCameraL,intrinsic_matrixL,distortion_matrixL,TagL.at(countofTag));
        });
        std::thread right=std::thread([&](){
            detectArucoTagPosition(FramefromCameraR,intrinsic_matrixR,distortion_matrixR,TagR.at(countofTag));
        });
        left.join();
        right.join();
//        detectArucoTagPosition(FramefromCameraL,intrinsic_matrixL,distortion_matrixL,TagL.at(countofTag));
//        detectArucoTagPosition(FramefromCameraR,intrinsic_matrixR,distortion_matrixR,TagR.at(countofTag));
        matchArucoTagFromSingalFrame(TagL.at(countofTag),TagR.at(countofTag));//matched the same id point of singal frame between right frame and left frame
        TriangulationRangingMethod();
        if(countofTag==(NumFrameForMean-1)){
            TransferFrameBasedDimension2MarkId();
            ProcessDimensionBasedMarkId();//get quaternion with matched frame from multiframe
            outputArucoPosture();
        }
        if(countofTag==(NumFrameForMean-1)){
            std::vector<Tag> add;
            add.resize(ArucoTagCount);
            TagL.erase(TagL.begin(),TagL.begin()+1);
            TagL.push_back(add);
            TagR.erase(TagR.begin(),TagR.begin()+1);
            TagR.push_back(add);
            DimensionBasedMarkIdTagL.clear();
            DimensionBasedMarkIdTagR.clear();
        }
        if(countofTag<(NumFrameForMean-1)){
            countofTag++;
        }
    } catch (...) {
//        cv::namedWindow("ReconstructArucoTagPose", 1);
//        imshow("ReconstructArucoTagPose", FramefromCameraL);
    }
}



void arucoPose::detectArucoTagPosition(cv::Mat& FrameDetect,
                                       cv::Mat& intrinsic_matrix,
                                       cv::Mat& distortion_matrix,
                                       std::vector<Tag>& singalFrameDetected){
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    std::vector<std::vector<cv::Point2f>> markCorners;
    std::vector<int> markIds;
    cv::Mat undistortedFrame;
    std::vector<cv::Vec3d> rvecs,tvecs;
    cv::aruco::detectMarkers(FrameDetect, dictionary, markCorners, markIds);
    cv::undistort(FrameDetect,undistortedFrame,intrinsic_matrix,distortion_matrix);
    FrameDetect=undistortedFrame.clone();
    cv::Mat undistortedDistortion_matrix=(cv::Mat_<double>(5,1)<<0,0,0,0,0);
    if(markIds.size()>0){
//        cv::aruco::drawDetectedMarkers(FrameDetect,markCorners,markIds);
        cv::aruco::estimatePoseSingleMarkers(markCorners,0.029,intrinsic_matrix,distortion_matrix,rvecs,tvecs);
        for(unsigned int i=0;i<markIds.size();i++){
            //          cv::aruco::drawAxis(FrameDetect,intrinsic_matrix,distortion_matrix,rvecs[i],tvecs[i],0.1);
            singalFrameDetected.at(i).rotVec=rvecs[i];
            singalFrameDetected.at(i).tvecs=tvecs[i];
            singalFrameDetected.at(i).markId=markIds.at(i);
            singalFrameDetected.at(i).markCorners=markCorners.at(i);
        }
    }

}


void arucoPose::matchArucoTagFromSingalFrame(std::vector<Tag>& RotVecofTagL,
                                             std::vector<Tag>& RotVecofTagR){
    float ForNoneImageInput=TagL.at(countofTag).at(0).markCorners.at(0).x;

    DimensionBasedMarkIdTagL.resize(ArucoTagCount);
    DimensionBasedMarkIdTagR.resize(ArucoTagCount);
    for(size_t i=0;i<RotVecofTagL.size();i++){
        if(RotVecofTagL.at(i).markId==0){
            RotVecofTagL.erase(RotVecofTagL.begin()+i,RotVecofTagL.begin()+i+1);
            if(i!=0)
                i--;
        }
        int flag=0;
        for(size_t j=0;j<RotVecofTagR.size();j++){
            if(RotVecofTagL.at(i).markId==RotVecofTagR.at(j).markId){
                flag=1;
                break;
            }
        }
        if(!flag){
            RotVecofTagL.erase(RotVecofTagL.begin()+i,RotVecofTagL.begin()+i+1);
            if(i!=0)
                i--;
        }
    }//matched same point for rotvecL
    for(size_t i=0;i<RotVecofTagR.size();i++){
        for(size_t j=0;j<RotVecofTagL.size();j++){
            if(RotVecofTagR.at(i).markId==RotVecofTagL.at(j).markId)
                break;
            if(j==(RotVecofTagL.size()-1)){
                RotVecofTagR.erase(RotVecofTagR.begin()+i,RotVecofTagR.begin()+i+1);
                i--;
            }
        }
    }//matched same point for rotvecR
}


void arucoPose::TriangulationRangingMethod(){
//    TagL.at(countofTag).resize(ArucoTagCount);
    for (size_t i=0;i<TagL.at(countofTag).size();i++){//changeable
        crossCenterPoint(TagL.at(countofTag).at(i));
        crossCenterPoint(TagR.at(countofTag).at(i));//center point position from singal frame with singal point
    }//center point position from singal frame
    cv::Mat projMatrixL,projMatrixR;
    cv::Mat points4D;
    projMatrixL=cv::Mat::zeros(3,4,CV_64F);
    projMatrixR=cv::Mat::zeros(3,4,CV_64F);
    intrinsic_matrixL.copyTo(projMatrixL(cv::Rect(0,0,3,3)));//transfer 3*3 intrinsMatrix to 3*4 form
    intrinsic_matrixR.copyTo(projMatrixR(cv::Rect(0,0,3,3)));


    projMatrixR=projMatrixR*transMatrixFromR2L;//put projectionMatrix in all

    for (size_t i=0;i<TagL.at(countofTag).size();i++){
        for(size_t j=0;j<TagR.at(countofTag).size();j++){
            if(TagL.at(countofTag).at(i).markId==TagR.at(countofTag).at(j).markId){
                cv::Mat projPointsL=(cv::Mat_<float>(2,1)<<TagL.at(countofTag).at(i).centerPoint.x,TagL.at(countofTag).at(i).centerPoint.y);
                cv::Mat projPointsR=(cv::Mat_<float>(2,1)<<TagR.at(countofTag).at(j).centerPoint.x,TagR.at(countofTag).at(j).centerPoint.y);
                cv::triangulatePoints(projMatrixL,projMatrixR,projPointsL,projPointsR,points4D);
//                if(points4D.at<float>(3,0)>1e-5){
                cv::Point3d point3d(points4D.at<float>(0,0)/points4D.at<float>(3,0),
                                    points4D.at<float>(1,0)/points4D.at<float>(3,0),
                                    points4D.at<float>(2,0)/points4D.at<float>(3,0));
                TagL.at(countofTag).at(i).reconstructCenterPoint=point3d;//only in TagL
//                }
                for(unsigned int k=0;k<4;k++){
                    cv::Mat projPointsL=(cv::Mat_<float>(2,1)<<TagL.at(countofTag).at(i).markCorners.at(k).x,TagL.at(countofTag).at(i).markCorners.at(k).y);
                    cv::Mat projPointsR=(cv::Mat_<float>(2,1)<<TagR.at(countofTag).at(j).markCorners.at(k).x,TagR.at(countofTag).at(j).markCorners.at(k).y);
                    cv::triangulatePoints(projMatrixL,projMatrixR,projPointsL,projPointsR,points4D);
//                    cv::convertPointsFromHomogeneous(points4D.t(),points3D);
//                    if(points4D.at<float>(3,0)>1e-5){
                    cv::Point3d point3d(points4D.at<float>(0,0)/points4D.at<float>(3,0),
                                        points4D.at<float>(1,0)/points4D.at<float>(3,0),
                                        points4D.at<float>(2,0)/points4D.at<float>(3,0));
                    TagL.at(countofTag).at(i).reconstructMarkCorners.push_back(point3d);//only in TagL
//                    }
                }
            }
        }
    }
}


void arucoPose::crossCenterPoint(Tag& TagForCrossCenter){
    std::vector<cv::Point3f> arucoTagCorner;//arucotag四个顶点
    cv::Point3f arucoTagCenterMeasured;//arucotag中心点坐标
    cv::Point2f currentPositionMeasured;//arucotag中心点二维坐标
    for(int j=0;j<4;j++)
        arucoTagCorner.push_back(cv::Point3f(TagForCrossCenter.markCorners.at(j).x,TagForCrossCenter.markCorners.at(j).y,1));

    arucoTagCenterMeasured = (arucoTagCorner.at(0).cross(arucoTagCorner.at(2))).cross((arucoTagCorner.at(1).cross(arucoTagCorner.at(3))));
    currentPositionMeasured.x = arucoTagCenterMeasured.x / arucoTagCenterMeasured.z;
    currentPositionMeasured.y = arucoTagCenterMeasured.y / arucoTagCenterMeasured.z;
    TagForCrossCenter.centerPoint=currentPositionMeasured;
}


void arucoPose::TransferFrameBasedDimension2MarkId(){
    for(size_t i=0;i<TagL.size();i++)
        for(size_t j=0;j<TagL.at(i).size();j++)
            TagL.at(i).at(j).frame=i;

    for(unsigned int i=1;i<=ArucoTagCount;i++)
        for(size_t j=0;j<TagL.size();j++)
            for(size_t k=0;k<TagL.at(j).size();k++){
                if(i==TagL.at(j).at(k).markId){
                    DimensionBasedMarkIdTagL.at(i-1).push_back(&TagL.at(j).at(k));
                }
                if(i==TagR.at(j).at(k).markId){
                    DimensionBasedMarkIdTagR.at(i-1).push_back(&TagR.at(j).at(k));
                }
            }
}

void arucoPose::ProcessDimensionBasedMarkId(){
    Eigen::Vector4d meanQuaternion;//the result
    for(size_t i=1;i<=ArucoTagCount;i++){
        unsigned int countofSameMarkId=DimensionBasedMarkIdTagL.at(i-1).size();
        if(countofSameMarkId>=MiniumNumFrameForMean){
            chooseStableArucoTag(DimensionBasedMarkIdTagL.at(i-1));//has choosed stable ArucoTag
            for(size_t j=0;j<DimensionBasedMarkIdTagL.at(i-1).size();j++){
                Eigen::Quaterniond quaternion(DimensionBasedMarkIdTagL.at(i-1).at(j)->PostureforQuatenionVector.rotation());//calculate many times
                DimensionBasedMarkIdTagL.at(i-1).at(j)->quaternionVector=quaternion;
            }//calculate the quaternion for id with enough frame
            AverageQuatertionfromMatchedMultiflame(DimensionBasedMarkIdTagL.at(i-1),meanQuaternion);
            for(size_t j=0;j<DimensionBasedMarkIdTagL.at(i-1).size();j++){
                if(DimensionBasedMarkIdTagL.at(i-1).at(j)->frame==(NumFrameForMean-1))
                    DimensionBasedMarkIdTagL.at(i-1).at(j)->averagequaternion=meanQuaternion;
            }
        }
    }
}

void arucoPose::chooseStableArucoTag(std::vector<Tag*>& ForChooseStableArucoTag){
    for (size_t i=0;i<ForChooseStableArucoTag.size();i++) {
        //        try {
        //            Eigen::Isometry3d a=ForChooseStableArucoTag.at(i)->PostureforQuatenionVector;
        //        } catch (...) {
        Eigen::Matrix3d reconstructR;
        Eigen::Isometry3d constructIsometry,cameraLeftIsometry;
        cv::Mat rotationMatrix;
        Eigen::Matrix3d eigenRotationMatrix;
        calculateNewCoordinatebyReconstructCorners(ForChooseStableArucoTag.at(i),reconstructR);
        constructIsometry=Eigen::Isometry3d::Identity();
        cameraLeftIsometry=Eigen::Isometry3d::Identity();
        constructIsometry.linear()=reconstructR;//check
        cv::Rodrigues(ForChooseStableArucoTag.at(i)->rotVec,rotationMatrix);
        cv::cv2eigen(rotationMatrix,eigenRotationMatrix);
        cameraLeftIsometry.linear()=eigenRotationMatrix;
        if(poseDiff_fromFrame1ToFrame2InBase(constructIsometry,cameraLeftIsometry)>20*M_PI/180)
            ForChooseStableArucoTag.at(i)->PostureforQuatenionVector=constructIsometry;
        else
            ForChooseStableArucoTag.at(i)->PostureforQuatenionVector=cameraLeftIsometry;
    }
}

void arucoPose::calculateNewCoordinatebyReconstructCorners(Tag* ForCalculateNewCoordinate,Eigen::Matrix3d& NewRotation){
    Eigen::Vector3d xAxis,yAxis,zAxis;

    xAxis.x()=(ForCalculateNewCoordinate->reconstructMarkCorners[1]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).x;
    xAxis.y()=(ForCalculateNewCoordinate->reconstructMarkCorners[1]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).y;
    xAxis.z()=(ForCalculateNewCoordinate->reconstructMarkCorners[1]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).z;
    yAxis.x()=(ForCalculateNewCoordinate->reconstructMarkCorners[3]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).x;
    yAxis.y()=(ForCalculateNewCoordinate->reconstructMarkCorners[3]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).y;
    yAxis.z()=(ForCalculateNewCoordinate->reconstructMarkCorners[3]-ForCalculateNewCoordinate->reconstructMarkCorners[0]).z;

    zAxis=yAxis.cross(xAxis);
    yAxis=xAxis.cross(zAxis);//solve the xAxis is not orthogonal with yAxis

    NewRotation.col(0)=xAxis.normalized();//or svd
    NewRotation.col(1)=yAxis.normalized();
    NewRotation.col(2)=zAxis.normalized();//normailize?

}

float arucoPose::poseDiff_fromFrame1ToFrame2InBase(Eigen::Isometry3d& _pose1InBase,
                                                   Eigen::Isometry3d& _pose2InBase){
    Eigen::Matrix<double,3,1> motionTrans;

    Eigen::AngleAxisd rotationFromPose1ToPose2InFrame1;
    rotationFromPose1ToPose2InFrame1.fromRotationMatrix(_pose1InBase.rotation().transpose()*_pose2InBase.rotation());
    motionTrans.matrix().block(0,0,3,1)=_pose1InBase.rotation()*rotationFromPose1ToPose2InFrame1.axis()*rotationFromPose1ToPose2InFrame1.angle();//_pose1InBase.rotation() is a  normalization matrixs//rotationFromPose1ToPose2InFrame1 may not be normalized
    //a rotation Matrix multipule a normal matrix still is a normal rotation Matrix
    double norm2=motionTrans.matrix().block(0,0,3,1).norm();//or calculate the angle of rotationFromPose1ToPose2InFrame1 immediately
    return norm2;
}


void arucoPose::AverageQuatertionfromMatchedMultiflame(std::vector<Tag*> BasedIdMarkTag,Eigen::Vector4d& averageResult){
    std::vector<Eigen::Vector4d> quaternions;
    quaternions.resize(BasedIdMarkTag.size());
    for(size_t i=0;i<BasedIdMarkTag.size();i++){
        quaternions.at(i)=BasedIdMarkTag.at(i)->quaternionVector.coeffs();
    }
    //    if (quaternions.size() == 0)
    //    {
    //        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
    //        return Eigen::Vector4f::Zero();
    //    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    for (size_t q=0; q<quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();
    // normalise with the number of quaternions
    A /= quaternions.size();
    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();
    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    double largestEigenValue;
    bool first = true;
    for (int i=0; i<singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }
    averageResult(0) = U(0, largestEigenValueIndex);
    averageResult(1) = U(1, largestEigenValueIndex);
    averageResult(2) = U(2, largestEigenValueIndex);
    averageResult(3) = U(3, largestEigenValueIndex);
}


void arucoPose::outputArucoPosture(){
    Eigen::Isometry3d ArucoTagPostureInCameraLeft=Eigen::Isometry3d::Identity();
    Eigen::Quaterniond empty(0,0,0,0);
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translate;
    std::vector<cv::Vec3d> rotationDraw,tevsDraw;

    int num2=0;
    std::vector<cv::Point2f> cornerL;
    int idL;

    for(size_t j=0;j<TagL.at(countofTag).size();j++){
        if(TagL.at(countofTag).at(j).averagequaternion==empty)
            continue;//test"is it empty?"
        else{
            rotation=TagL.at(countofTag).at(j).averagequaternion.toRotationMatrix();
            ArucoTagPostureInCameraLeft.linear()=rotation;
            translate=Eigen::Vector3d(TagL.at(countofTag).at(j).reconstructCenterPoint.x,TagL.at(countofTag).at(j).reconstructCenterPoint.y,TagL.at(countofTag).at(j).reconstructCenterPoint.z);
            ArucoTagPostureInCameraLeft.translation()=translate;
            Eigen::AngleAxisd axisangle(rotation);
            cv::Vec3d newRotVec(axisangle.axis()[0]*axisangle.angle(),axisangle.axis()[1]*axisangle.angle(),axisangle.axis()[2]*axisangle.angle());
            cv::Vec3d newTvecs(translate[0],translate[1],translate[2]);
            rotationDraw.push_back(newRotVec);
            tevsDraw.push_back(newTvecs/1000);
            output.at(0).outputObject.objectMarkId=TagL.at(countofTag).at(j).markId;
            output.at(0).outputObject.objectPosture=ArucoTagPostureInCameraLeft;
            for(size_t i=0;i<TagL.at(countofTag).size();i++){
                std::vector<std::vector<cv::Point2f>> testmarkCorners;
                std::vector<int> testmarkIds;
                //        if(TagL.at(countofTag).at(i).markId==20){
                //            cv::circle(FramefromCameraL,TagL.at(countofTag).at(i).centerPoint,5,cv::Scalar(0,0,255),-1);
                //            std::cout<<TagL.at(countofTag).at(i).markCorners.at(0).x<<", "<<TagL.at(countofTag).at(i).markCorners.at(0).y<<", ";
                //            std::cout<<TagL.at(countofTag).at(i).centerPoint.x<<", "<<TagL.at(countofTag).at(i).centerPoint.y<<std::endl;
                //        }
                double distance20,distance21;
                double disX1,disX2,disY1,disY2,disZ1,disZ2;

//if(TagL.at(countofTag).at(i).markId==ObjectForDetecting["objectmarkId"].as<unsigned int>()){
//    output.at(ObjectForDetecting["objectId"].as<int>()).objectId=ObjectForDetecting["objectId"].as<int>();
// output.at(ObjectForDetecting["objectId"].as<int>()).outputObject.objectMarkId=TagL.at(countofTag).at(j).markId;
//  output.at(ObjectForDetecting["objectId"].as<int>()).outputObject.objectPosture=ArucoTagPostureInCameraLeft;
//  YAML::Node object1=ObjectForDetecting["object1"];
//  const YAML::Node& sizeNode=object1["shape_size"];
//  float size[3];
//  if(sizeNode.IsSequence()){
//      YAML::const_iterator it=sizeNode.begin();
////      std::vector<float> sizeVector;
////      for(const YAML::Node size:sizeNode)
////          sizeVector.push_back(size.as<float>());

//      for(unsigned int i=0;i<sizeNode.size();i++){
//          size[i]=it->as<float>();
//          output.at(ObjectForDetecting["objectId"].as<int>()).shape_size_obj[i]=it->as<float>();
//          it++;
//      }
//  }
//}
                if(TagL.at(countofTag).at(i).markId==20){
                    //            std::cout<<TagL.at(countofTag).at(i).centerPoint<<";"<<TagR.at(countofTag).at(i).centerPoint<<std::endl;
                    disX1=TagL.at(countofTag).at(i).reconstructCenterPoint.x;
                    disY1=TagL.at(countofTag).at(i).reconstructCenterPoint.y;
                    disZ1=TagL.at(countofTag).at(i).reconstructCenterPoint.z;
                    distance20=sqrt(disX1*disX1+disY1*disY1+disZ1*disZ1);
                    cornerL=TagL.at(countofTag).at(i).markCorners;
                    idL=TagL.at(countofTag).at(i).markId;

                    num2=1;


                }
                if(TagL.at(countofTag).at(i).markId==21&&num2){
                    disX2=TagL.at(countofTag).at(i).reconstructCenterPoint.x;
                    disY2=TagL.at(countofTag).at(i).reconstructCenterPoint.y;
                    disZ2=TagL.at(countofTag).at(i).reconstructCenterPoint.z;
                    distance21=sqrt(disX2*disX2+disY2*disY2+disZ2*disZ2);
                    testmarkIds.push_back(idL);
                    testmarkCorners.push_back(cornerL);
//                    std::cout<<distance20<<", ";
//                    std::cout<<distance21<<", ";
                    testmarkCorners.push_back(TagL.at(countofTag).at(i).markCorners);
                    testmarkIds.push_back(TagL.at(countofTag).at(i).markId);
                    double distance=sqrt((disX2-disX1)*(disX2-disX1)+(disY2-disY1)*(disY2-disY1)+(disZ1-disZ2)*(disZ1-disZ2));
//                    std::cout<<distance<<std::endl;

                }
                cv::aruco::drawDetectedMarkers(FramefromCameraL,testmarkCorners,testmarkIds);

            }



        }
        for(size_t i=0;i<rotationDraw.size();i++){
            //                            cv::aruco::drawDetectedMarkers(FramefromCameraL,TagL.at(countofTag).at(j).markCorners,TagL.at(countofTag).at(j).markId);
            cv::aruco::drawAxis(FramefromCameraL,intrinsic_matrixL,distortion_matrixL,rotationDraw[i],tevsDraw[i],0.05);
        }
//        cv::namedWindow("ReconstructArucoTagPose", 1);
//        imshow("ReconstructArucoTagPose", FramefromCameraL);
        //    cv::waitKey(0);

    }
}
