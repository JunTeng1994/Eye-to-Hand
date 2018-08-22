#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

bool SaveMatrix(string fileName, cv::Mat R, cv::Mat t)
{
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        return false;
    }

    cv::Mat transMat=cv::Mat::eye(cv::Size(4, 4), CV_32F);
    R.copyTo(transMat(cv::Rect(0, 0, 3, 3)));
    t.copyTo(transMat(cv::Rect(3, 0, 1, 3)));
    fs << "TransMat" << transMat;
    fs.release();

    return true;
}

bool ReadMatrix(string fileName, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distortionCoefficients"] >> distCoeffs;
    cout << "cameraMatrix: " << endl << cameraMatrix << endl
         << "distortionCoefficients: " << endl << distCoeffs << endl;
    fs.release();

    return true;
}

int main()
{
    cv::Mat ImageIn = cv::imread("./data/image0.jpg",0);
    if (!ImageIn.data)
    {
        cout << "Fail to read image ..." << endl;
        return -1;
    }

    cv::Size patternsize(3,5);
    vector<cv::Point2f> centres;
    cv::SimpleBlobDetector::Params blobParams;
    blobParams.filterByArea = true;
    blobParams.minArea = 0;
    blobParams.maxArea = 60;
    blobParams.filterByColor = true;
    blobParams.blobColor = 255;
    blobParams.filterByCircularity = true;
    blobParams.minCircularity = 0.5;
    blobParams.filterByConvexity = true;
    blobParams.minConvexity = 0.5;
    blobParams.maxConvexity = 100.0;
    blobParams.filterByInertia = true;
    blobParams.minInertiaRatio = 0.1;
    blobParams.maxInertiaRatio = 100.0;
    blobParams.thresholdStep = 60;
    cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(blobParams);
    bool patterfound = cv::findCirclesGrid(ImageIn, patternsize, centres, cv::CALIB_CB_ASYMMETRIC_GRID, blobDetector);

    vector<cv::Point3f> object(15);
    for (int i = 0; i < 5; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            object[i * 3 + j].x = 0.1*j + 0.05*(i%2);
            object[i * 3 + j].y = 0.05*i;
            object[i * 3 + j].z = 0.0;
            cout << object[i * 3 + j].x << "  " << object[i * 3 + j].y << "  "<< object[i * 3 + j].z << endl;
        }
    }
    cv::Mat cameraMatrix(3, 3, CV_32F);
    cv::Mat distCoeffs;
    if (!ReadMatrix("./data/camera_ir_kinect2.yaml", cameraMatrix, distCoeffs))
    {
        cout << "Fail to read matrix ..." << endl;
        return -1;
    }

    cv::Mat rvec, tvec;
    cv::Mat transMat(3, 3, CV_32F);
    cv::solvePnP(object, centres, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, transMat);

    std::vector<cv::Point3f> ObjectPoints(4);
    ObjectPoints[0].x = 0;  ObjectPoints[0].y = 0;  ObjectPoints[0].z = 0;
    ObjectPoints[1].x = 1;  ObjectPoints[1].y = 0;  ObjectPoints[1].z = 0;
    ObjectPoints[2].x = 0;  ObjectPoints[2].y = 1;  ObjectPoints[2].z = 0;
    ObjectPoints[3].x = 0;  ObjectPoints[3].y = 0;  ObjectPoints[3].z = 1;
    std::vector<cv::Point2f> ImagePoints(4);
    cv::projectPoints(ObjectPoints,rvec,tvec, cameraMatrix,distCoeffs,ImagePoints);

    cv::line(ImageIn, ImagePoints[0], ImagePoints[1], cv::Scalar(255));
    cv::line(ImageIn, ImagePoints[0], ImagePoints[2], cv::Scalar(255));
    cv::line(ImageIn, ImagePoints[0], ImagePoints[3], cv::Scalar(255));
    cout << ImagePoints[3].x-ImagePoints[0].x << "  " << ImagePoints[3].y - ImagePoints[0].y << "  " << endl;
    //cv::drawChessboardCorners(ImageIn, patternsize, cv::Mat(centres), patterfound);
    cv::imshow("ImageShow", ImageIn);

    for(int i = 0; i < 2 ;i++){
        for(int j = 0; j < 3; j++){
            transMat.at<float>(i,j) = -transMat.at<float>(i,j);
        }
        tvec.at<float>(i) = -tvec.at<float>(i);
    }
    cout << "Rotation matrix:" << transMat << endl;
    cout << "Transform vector:" << tvec << endl;

    if (!SaveMatrix("./data/A0.yaml", transMat, tvec))
    {
        cout << "Fail to write matrix" << endl;
        return -1;
    }

    /*vector<cv::Point2f> centresy;
    for (int i = 0;i < 12;i++)
    {
        centresy.push_back(centres[i]);
    }*/

    cvWaitKey();

    return 0;
}