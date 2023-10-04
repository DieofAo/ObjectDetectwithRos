#include "objectDetectorViaAruco.h"


arucoPose::~arucoPose(){

}

arucoPose::arucoPose(std::string& configYaml,std::vector<struct msgPose>& outputObjectInformation){
//    distortion_matrixL=(cv::Mat_<double>(5,1)<<0.0128,-0.0343,0.0023,-0.00011,0);
//    intrinsic_matrixL=(cv::Mat_<double>(3,3)<<921.6992,0,653.3826,
//                         0,918.4418,364.1299,
//                         0,0,1);
//    distortion_matrixR=(cv::Mat_<double>(5,1)<<0.0135,-0.0339,0.0032,-0.00011,0);
//    intrinsic_matrixR=(cv::Mat_<double>(3,3)<<920.5909,0,646.5033,
//                         0,917.4791,370.2785,
//                         0,0,1);
//    transMatrixFromR2L=(cv::Mat_<double>(4,4)<<1.0,0.0023,1.4284e-4,-65.2972,
//                          -0.0023,1.0,-0.0066,0.0304,
//                          1.5806e-4,0.0066,1.0,-0.1852,
//                          0,0,0,1);



    TagL.resize(NumFrameForMean,std::vector<Tag>(ArucoTagCount));
    TagR.resize(NumFrameForMean,std::vector<Tag>(ArucoTagCount));
    DimensionBasedMarkIdTagL.resize(ArucoTagCount);
    DimensionBasedMarkIdTagR.resize(ArucoTagCount);

    ObjectForDetecting=YAML::LoadFile(configYaml);

    output.resize(ObjectForDetecting["objectNum"].as<unsigned int>());

    _outputObjectInformation=&outputObjectInformation;

}
void arucoPose::configCameraInformation(int cameraId){
    YAML::Node configCamera;
    if(cameraId==ObjectForDetecting["handCameraId"].as<int>())
        configCamera=YAML::LoadFile("/home/jqzlca/workspace/catkin_ws/src/jqz_ws/object_detector/config/HandCamera.yaml");
    if(cameraId==ObjectForDetecting["globalCameraId"].as<int>())
        configCamera=YAML::LoadFile("/home/jqzlca/workspace/catkin_ws/src/jqz_ws/object_detector/config/GlobalCamera.yaml");
    const YAML::Node& sizeNodeL=configCamera["distortionMatrixL"];
    float sizeL[5];
    if(sizeNodeL.IsSequence()){
        YAML::const_iterator it=sizeNodeL.begin();
        for(unsigned int i=0;i<sizeNodeL.size();i++){
            sizeL[i]=it->as<float>();
            it++;
        }
    }
    distortion_matrixL=(cv::Mat_<double>(5,1)<<sizeL[0],sizeL[1],sizeL[2],sizeL[3],sizeL[4]);

    const YAML::Node& sizeNodeR=configCamera["distortionMatrixR"];
    float sizeR[5];
    if(sizeNodeR.IsSequence()){
        YAML::const_iterator it=sizeNodeR.begin();
        for(unsigned int i=0;i<sizeNodeR.size();i++){
            sizeR[i]=it->as<float>();
            it++;
        }
    }
    distortion_matrixR=(cv::Mat_<double>(5,1)<<sizeR[0],sizeR[1],sizeR[2],sizeR[3],sizeR[4]);

    float intrinRMatrix[3][3];
    YAML::Node intrinR=configCamera["intrinsicMatrixR"]["first"];
    const YAML::Node& intrinRFirst=intrinR;
    if(intrinRFirst.IsSequence()){
        YAML::const_iterator it=intrinRFirst.begin();
        for(unsigned int i=0;i<intrinRFirst.size();i++){
            intrinRMatrix[0][i]=it->as<float>();
            it++;
        }
    }
    intrinR=configCamera["intrinsicMatrixR"]["second"];
    const YAML::Node& intrinRSecond=intrinR;
    if(intrinRSecond.IsSequence()){
        YAML::const_iterator it=intrinRSecond.begin();
        for(unsigned int i=0;i<intrinRSecond.size();i++){
            intrinRMatrix[1][i]=it->as<float>();
            it++;
        }
    }
    intrinR=configCamera["intrinsicMatrixR"]["third"];
    const YAML::Node& intrinRThird=intrinR;
    if(intrinRThird.IsSequence()){
        YAML::const_iterator it=intrinRThird.begin();
        for(unsigned int i=0;i<intrinRThird.size();i++){
            intrinRMatrix[2][i]=it->as<float>();
            it++;
        }
    }
    intrinsic_matrixR=(cv::Mat_<double>(3,3)<<intrinRMatrix[0][0],intrinRMatrix[0][1],intrinRMatrix[0][2],
                         intrinRMatrix[1][0],intrinRMatrix[1][1],intrinRMatrix[1][2],
                         intrinRMatrix[2][0],intrinRMatrix[2][1],intrinRMatrix[2][2]);

    float intrinLMatrix[3][3];
    YAML::Node intrinL=configCamera["intrinsicMatrixL"]["first"];
    const YAML::Node& intrinLFirst=intrinL;
    if(intrinLFirst.IsSequence()){
        YAML::const_iterator it=intrinLFirst.begin();
        for(unsigned int i=0;i<intrinLFirst.size();i++){
            intrinLMatrix[0][i]=it->as<float>();
            it++;
        }
    }
    intrinL=configCamera["intrinsicMatrixL"]["second"];
    const YAML::Node& intrinLSecond=intrinL;
    if(intrinLSecond.IsSequence()){
        YAML::const_iterator it=intrinLSecond.begin();
        for(unsigned int i=0;i<intrinLSecond.size();i++){
            intrinLMatrix[1][i]=it->as<float>();
            it++;
        }
    }
    intrinL=configCamera["intrinsicMatrixL"]["third"];
    const YAML::Node& intrinLThird=intrinL;
    if(intrinLThird.IsSequence()){
        YAML::const_iterator it=intrinLThird.begin();
        for(unsigned int i=0;i<intrinLThird.size();i++){
            intrinLMatrix[2][i]=it->as<float>();
            it++;
        }
    }
    intrinsic_matrixL=(cv::Mat_<double>(3,3)<<intrinLMatrix[0][0],intrinLMatrix[0][1],intrinLMatrix[0][2],
                         intrinLMatrix[1][0],intrinLMatrix[1][1],intrinLMatrix[1][2],
                         intrinLMatrix[2][0],intrinLMatrix[2][1],intrinLMatrix[2][2]);

    float transMatrix[3][4];
    YAML::Node Trans=configCamera["transMatrix"]["first"];
    const YAML::Node& TransFirst=Trans;
    if(TransFirst.IsSequence()){
        YAML::const_iterator it=TransFirst.begin();
        for(unsigned int i=0;i<TransFirst.size();i++){
            transMatrix[0][i]=it->as<float>();
            it++;
        }
    }
    Trans=configCamera["transMatrix"]["second"];
    const YAML::Node& TransSecond=Trans;
    if(TransSecond.IsSequence()){
        YAML::const_iterator it=TransSecond.begin();
        for(unsigned int i=0;i<TransSecond.size();i++){
            transMatrix[1][i]=it->as<float>();
            it++;
        }
    }
    Trans=configCamera["transMatrix"]["third"];
    const YAML::Node& TransThird=Trans;
    if(TransThird.IsSequence()){
        YAML::const_iterator it=TransThird.begin();
        for(unsigned int i=0;i<TransThird.size();i++){
            transMatrix[2][i]=it->as<float>();
            it++;
        }
    }
    transMatrixFromR2L=(cv::Mat_<double>(4,4)<<transMatrix[0][0],transMatrix[0][1],transMatrix[0][2],transMatrix[0][3],
                          transMatrix[1][0],transMatrix[1][1],transMatrix[1][2],transMatrix[1][3],
                          transMatrix[2][0],transMatrix[2][1],transMatrix[2][2],transMatrix[2][3],
                          0,0,0,1);



    projMatrixL=cv::Mat::zeros(3,4,CV_64F);
    projMatrixR=cv::Mat::zeros(3,4,CV_64F);
    intrinsic_matrixL.copyTo(projMatrixL(cv::Rect(0,0,3,3)));//transfer 3*3 intrinsMatrix to 3*4 form
    intrinsic_matrixR.copyTo(projMatrixR(cv::Rect(0,0,3,3)));
    projMatrixR=projMatrixR*transMatrixFromR2L;//changable

}
void arucoPose::runDetectArucoTagPosByStereoCamera(cv::Mat& FrameDetectL,
                                                   cv::Mat& FrameDetectR,
                                                   int cameraId){
    configCameraInformation(cameraId);
    FramefromCameraL=FrameDetectL.clone();
    FramefromCameraR=FrameDetectR.clone();
    std::thread left=std::thread([&](){
        detectArucoTagPosition(FramefromCameraL,intrinsic_matrixL,distortion_matrixL,TagL.at(countofTag));
    });
    std::thread right=std::thread([&](){
        detectArucoTagPosition(FramefromCameraR,intrinsic_matrixR,distortion_matrixR,TagR.at(countofTag));
    });
    left.join();
    right.join();
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
            //                      cv::aruco::drawAxis(FrameDetect,intrinsic_matrix,distortion_matrix,rvecs[i],tvecs[i],0.1);
            singalFrameDetected.at(i).rotVec=rvecs[i];
            singalFrameDetected.at(i).tvecs=tvecs[i];
            singalFrameDetected.at(i).markId=markIds.at(i);
            singalFrameDetected.at(i).markCorners=markCorners.at(i);
        }
    }
    if(markIds.size()==0){
        singalFrameDetected.clear();

    }

}


void arucoPose::matchArucoTagFromSingalFrame(std::vector<Tag>& RotVecofTagL,
                                             std::vector<Tag>& RotVecofTagR){
    DimensionBasedMarkIdTagL.clear();
    DimensionBasedMarkIdTagR.clear();
    DimensionBasedMarkIdTagL.resize(ArucoTagCount);
    DimensionBasedMarkIdTagR.resize(ArucoTagCount);//changeable the position
    for(int i=0;i<RotVecofTagL.size();i++){
        if(RotVecofTagL.at(i).markId==0){
            RotVecofTagL.erase(RotVecofTagL.begin()+i,RotVecofTagL.begin()+i+1);
            if(i!=0)
                i--;
        }
    }
    for(int i=0;i<RotVecofTagR.size();i++){
        if(RotVecofTagR.at(i).markId==0){
            RotVecofTagR.erase(RotVecofTagR.begin()+i,RotVecofTagR.begin()+i+1);
            if(i!=0)
                i--;
        }
    }//changable


    for(int i=0;i<RotVecofTagL.size();i++){

        int flag=0;

        for(size_t j=0;j<RotVecofTagR.size();j++){
            if(RotVecofTagL.at(i).markId==RotVecofTagR.at(j).markId){
                flag=1;
                break;
            }
        }

        if(!flag){
            RotVecofTagL.erase(RotVecofTagL.begin()+i,RotVecofTagL.begin()+i+1);

            i--;
        }
    }//matched same point for rotvecL
    if(RotVecofTagL.size()){
        for(int i=0;i<RotVecofTagR.size();i++){
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
    else{
        RotVecofTagR.clear();
    }
}


void arucoPose::TriangulationRangingMethod(){
    //    TagL.at(countofTag).resize(ArucoTagCount);
    for (size_t i=0;i<TagL.at(countofTag).size();i++){//changeable
        crossCenterPoint(TagL.at(countofTag).at(i));
        crossCenterPoint(TagR.at(countofTag).at(i));//center point position from singal frame with singal point
    }//center point position from singal frame
    cv::Mat points4D;

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
    if(quaternions.size()!=0){
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
}


void arucoPose::outputArucoPosture(){
    Eigen::Isometry3d ArucoTagPostureInCameraLeft=Eigen::Isometry3d::Identity();

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translate;
    std::vector<cv::Vec3d> rotationDraw,tevsDraw;

    output.clear();
    _outputObjectInformation->clear();
    output.resize(ObjectForDetecting["objectNum"].as<unsigned int>());


    std::vector<std::vector<cv::Point2f>> testmarkCorners;
    std::vector<int> testmarkIds;


    for(size_t j=0;j<TagL.at(countofTag).size();j++){
        if(fabs(TagL.at(countofTag).at(j).averagequaternion.norm()-1)>1e-5)
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
            tevsDraw.push_back(newTvecs/1000);//transfer unit from mm to m


            YAML::Node object;
            for(int jj=0;jj<ObjectForDetecting["objectNum"].as<int>();jj++){
                std::string objectForYaml="object"+std::to_string(jj+1);
                for(int i=0;i<ObjectForDetecting[objectForYaml]["numofmark"].as<int>();i++){
                    std::string objectmarkIdForYaml="objectmarkId"+std::to_string(i);
                    if(TagL.at(countofTag).at(j).markId==ObjectForDetecting[objectForYaml][objectmarkIdForYaml].as<unsigned int>()){
                        object=ObjectForDetecting[objectForYaml];
                        jj=ObjectForDetecting["objectNum"].as<int>();//just for exit the first circulation
                        break;
                    }
                }
            }
            output.at(object["objectId"].as<int>()).objectId=object["objectId"].as<int>();
            output.at(object["objectId"].as<int>()).outputObject.objectMarkId=TagL.at(countofTag).at(j).markId;
            output.at(object["objectId"].as<int>()).outputObject.objectTag=TagL.at(countofTag).at(j);
            output.at(object["objectId"].as<int>()).outputObject.objectPosture=ArucoTagPostureInCameraLeft;



            const YAML::Node& sizeNode=object["shape_size"];
            float size[3];
            if(sizeNode.IsSequence()){
                YAML::const_iterator it=sizeNode.begin();
                //      std::vector<float> sizeVector;
                //      for(const YAML::Node size:sizeNode)
                //          sizeVector.push_back(size.as<float>());

                for(unsigned int i=0;i<sizeNode.size();i++){
                    size[i]=it->as<float>();
                    output.at(object["objectId"].as<int>()).shape_size_obj[i]=it->as<float>();
                    it++;
                }
                //Unify to the same code Cooridante by rotation by translation
                Eigen::Isometry3d transferArucotagToObject;
                transferArucotagToObject.setIdentity();
                Eigen::Vector3d linear;
                linear<<0,0,-output.at(object["objectId"].as<int>()).shape_size_obj[0]/2.0;
                transferArucotagToObject.translation()=linear;
                output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object=output.at(object["objectId"].as<int>()).outputObject.objectPosture*transferArucotagToObject;
            }


            //Unify to the same code Cooridante by rotation
            int i=TagL.at(countofTag).at(j).markId-object["objectmarkId0"].as<int>();
            Eigen::Isometry3d rotation2leastMarkPose;
            rotation2leastMarkPose.setIdentity();

            if(i){

                Eigen::AngleAxisd rotation_vectoryY;
                if(object["objectId"].as<int>()==0)
                rotation_vectoryY=Eigen::AngleAxisd(-M_PI*i/2,Eigen::Vector3d(0,1,0));
                if(object["objectId"].as<int>()==1)
                rotation_vectoryY=Eigen::AngleAxisd(-M_PI*i/3,Eigen::Vector3d(0,1,0));
                Eigen::Matrix3d rotation_vectorYMatrix=rotation_vectoryY.matrix();
                rotation2leastMarkPose.rotate(rotation_vectorYMatrix);
//                output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object*rotation2leastMarkPose;
            }

            //rotate cooridantion from Tag detected result frame to object pose in plan frame
            Eigen::Isometry3d rotation2MarkPoseInPlanSence;
            rotation2MarkPoseInPlanSence.setIdentity();
            Eigen::AngleAxisd rotation_vectoryX(-M_PI/2,Eigen::Vector3d(1,0,0));
            Eigen::Matrix3d rotation_vectorXMatrix=rotation_vectoryX.matrix();
            rotation2MarkPoseInPlanSence.rotate(rotation_vectorXMatrix);


            Eigen::Isometry3d rotation2MarkPoseXYZNormal;
            rotation2MarkPoseXYZNormal.setIdentity();
            Eigen::AngleAxisd rotation_vectoryZ(-M_PI/2,Eigen::Vector3d(0,0,1));
            Eigen::Matrix3d rotation_vectorXYZMatrix=rotation_vectoryZ.matrix();
            rotation2MarkPoseXYZNormal.rotate(rotation_vectorXYZMatrix);
//            output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object*rotation2MarkPoseXYZNormal*rotation2MarkPoseInPlanSence*rotation2leastMarkPose;
            output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object*rotation2leastMarkPose*rotation2MarkPoseInPlanSence*rotation2MarkPoseXYZNormal;

            //config the outPut msg information of objectPose with messageRos__ever frame ever object

            output.at(object["objectId"].as<int>()).outputObject.outputObjectPose=Eigen::Quaterniond(output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.rotation());
            struct msgPose objectForMsg;
            objectForMsg.pose.outputObjectPose=output.at(object["objectId"].as<int>()).outputObject.outputObjectPose;


            output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().x()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().x()/1000.0;
            output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().y()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().y()/1000.0;
            output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().z()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().z()/1000.0;
            objectForMsg.pose.tranformationFromTag2Object.translation().x()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().x();
            objectForMsg.pose.tranformationFromTag2Object.translation().y()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().y();
            objectForMsg.pose.tranformationFromTag2Object.translation().z()=output.at(object["objectId"].as<int>()).outputObject.tranformationFromTag2Object.translation().z();


            objectForMsg.objectId=object["objectId"].as<int>();
            _outputObjectInformation->push_back(objectForMsg);

            //draw the axis and detected ArucoTag
            testmarkCorners.push_back(output.at(object["objectId"].as<int>()).outputObject.objectTag.markCorners);
            testmarkIds.push_back(output.at(object["objectId"].as<int>()).outputObject.objectTag.markId);

        }

    }
    cv::aruco::drawDetectedMarkers(FramefromCameraL,testmarkCorners,testmarkIds);

    for(size_t i=0;i<rotationDraw.size();i++){
        //                            cv::aruco::drawDetectedMarkers(FramefromCameraL,TagL.at(countofTag).at(j).markCorners,TagL.at(countofTag).at(j).markId);
        cv::aruco::drawAxis(FramefromCameraL,intrinsic_matrixL,distortion_matrixL,rotationDraw[i],tevsDraw[i],0.05);
//        std::cout<<tevsDraw[i]<<std::endl;
    }
//                cv::namedWindow("testFrame", 1);
//                imshow("testFrame", FramefromCameraL);
}
