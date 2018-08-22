#include "stdafx.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>



class CLightSpotDetector
{

public:

    CLightSpotDetector(std::string str = "");
    ~CLightSpotDetector();

    void SetBlobDetectorParams();

    void DetectBlobKeyPoints(cv::Mat inputImg);

    void WriteBlobDetectorParams(std::string str);

    void ReadBlobDetectorParams(std::string str);

    void RecognizeAndEncode();



    cv::Ptr<cv::SimpleBlobDetector> blobDetector;
    std::vector<cv::KeyPoint> blobKeyPoints;
    std::vector<cv::KeyPoint> goodKeyPoints;
    std::vector<cv::Point2f> goodPoints;
    cv::SimpleBlobDetector::Params blobParams;
    cv::Mat blobImg;
    cv::Mat detectImg;

};