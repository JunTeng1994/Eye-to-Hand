#include "lightSpot.h"



void CLightSpotDetector::SetBlobDetectorParams()
{
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
}

void CLightSpotDetector::WriteBlobDetectorParams(std::string str)
{
    cv::FileStorage fs(str, cv::FileStorage::WRITE);
    blobParams.write(fs);
    std::cout << "Write blob params file." << std::endl;
    fs.release();

}

void CLightSpotDetector::ReadBlobDetectorParams(std::string str)
{
    cv::FileStorage fs(str, cv::FileStorage::READ);
    //blobParams.read(fs["thresholdStep"]);
    blobParams.read(fs[""]);
    //std::cout << blobParams.minCircularity << std::endl;
    //blobParams.read(fs["minThreshold"]);
    //blobParams.read(fs["maxThreshold"]);
    //blobParams.read(fs["minRepeatability"]);
    //blobParams.read(fs["minDistBetweenBlobs"]);
    //blobParams.read(fs["filterByColor"]);
    //blobParams.read(fs["blobColor"]);
    //blobParams.read(fs["filterByArea"]);
    //blobParams.read(fs["minArea"]);
    //blobParams.read(fs["maxArea"]);
    //blobParams.read(fs["filterByCircularity"]);
    //blobParams.read(fs["minCircularity"]);
    //blobParams.read(fs["maxCircularity"]);
    //blobParams.read(fs["filterByInertial"]);
    //blobParams.read(fs["minInertiaRatio"]);
    //blobParams.read(fs["maxInertiaRatio"]);
    //blobParams.read(fs["filterByConvexity"]);
    //blobParams.read(fs["minConvexity"]);
    //blobParams.read(fs["maxConvexity"]);
    
    std::cout << "Read blob params file: " << str << std::endl;
    
    fs.release();

}

CLightSpotDetector::CLightSpotDetector(std::string str)
{
    if (str == "")
    {
        SetBlobDetectorParams();
    }
    else
    { 
        ReadBlobDetectorParams(str);
    }

    blobDetector = cv::SimpleBlobDetector::create(blobParams);
}

CLightSpotDetector::~CLightSpotDetector()
{

}
void CLightSpotDetector::DetectBlobKeyPoints(cv::Mat inputImg)
{
    blobKeyPoints.clear();// clear element, but not clear the capacity.
    goodKeyPoints.clear();
    goodPoints.clear();
    cv::resize(inputImg, detectImg, inputImg.size());
    blobDetector->detect(detectImg, blobKeyPoints);

    //std::cout << "Detect " << blobKeyPoints.size() << " light spots." << std::endl;

    cv::drawKeypoints(detectImg, blobKeyPoints, blobImg, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    RecognizeAndEncode();
}

void CLightSpotDetector::RecognizeAndEncode()
{
    if (blobKeyPoints.size() >= 3)
    {
        cv::Vec4f line;
        std::vector<cv::Point2f> inputLinePt;
        std::vector<cv::Point2f> linePoints;
        for (int i = 0; i < blobKeyPoints.size(); i++)
        {
            inputLinePt.push_back(blobKeyPoints[i].pt);
        }
        cv::fitLine(inputLinePt, line, CV_DIST_HUBER, 0, 0.01, 0.01);
        double k_ = line[1] / line[0];
        double b_ = line[3] - (k_ * line[2]);
        for (int i = 0; i < blobKeyPoints.size(); i++)
        {
            cv::Point2f pt_ = blobKeyPoints[i].pt;
            if ((pt_.x * k_ + b_ - pt_.y) < 5)
            {
                linePoints.push_back(pt_);
            }
        }
        if (linePoints.size() > 1)
        {
            cv::line(blobImg, linePoints[0], linePoints[1], cv::Scalar(255, 0, 0), 0.5);
        }
    }
    
    if (blobKeyPoints.size() == 3)
    {
        cv::Point2f pt;
        double dis[3];
        pt = (blobKeyPoints[0].pt - blobKeyPoints[1].pt);
        dis[0] = sqrt(pt.ddot(pt));
        pt = (blobKeyPoints[1].pt - blobKeyPoints[2].pt);
        dis[1] = sqrt(pt.ddot(pt));
        pt = (blobKeyPoints[2].pt - blobKeyPoints[0].pt);
        dis[2] = sqrt(pt.ddot(pt));
        double minDis = std::min(std::min(dis[0], dis[1]), dis[2]);
        //std::cout << dis[0] << std::endl;
        //std::cout << dis[1] << std::endl;
        //std::cout << dis[2] << std::endl; 
        if (minDis == dis[0])
        {
            if (dis[1] < dis[2])
            {
                blobKeyPoints[0].class_id = 0;
                blobKeyPoints[1].class_id = 1;
                blobKeyPoints[2].class_id = 2;
            }
            else
            {
                blobKeyPoints[0].class_id = 1;
                blobKeyPoints[1].class_id = 0;
                blobKeyPoints[2].class_id = 2;
            }
        }
        else if (minDis == dis[1])
        {
            if (dis[0] < dis[2])
            {
                blobKeyPoints[0].class_id = 2;
                blobKeyPoints[1].class_id = 1;
                blobKeyPoints[2].class_id = 0;
            }
            else
            {
                blobKeyPoints[0].class_id = 2;
                blobKeyPoints[1].class_id = 0;
                blobKeyPoints[2].class_id = 1;
            }
        }
        else if (minDis == dis[2])
        {
            if (dis[0] < dis[1])
            {
                blobKeyPoints[0].class_id = 1;
                blobKeyPoints[1].class_id = 2;
                blobKeyPoints[2].class_id = 0;
            }
            else
            {
                blobKeyPoints[0].class_id = 0;
                blobKeyPoints[1].class_id = 2;
                blobKeyPoints[2].class_id = 1;
            }

        }
        for (int i = 0; i < 3; i++)
        {
            //cv::QtFont font = cv::fontQt("Times");
            if (blobKeyPoints[i].class_id == 0)
            {
                //cv::addText(blobImg, "0", blobKeyPoints[i].pt, font);
                cv::putText(blobImg, "0", blobKeyPoints[i].pt, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            }
            if (blobKeyPoints[i].class_id == 1)
            {
                cv::putText(blobImg, "1", blobKeyPoints[i].pt, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            }
            if (blobKeyPoints[i].class_id == 2)
            {
                cv::putText(blobImg, "2", blobKeyPoints[i].pt, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
            }
        }

        if (blobKeyPoints[0].class_id == 0)
        {
            goodKeyPoints.push_back(blobKeyPoints[0]);
            if (blobKeyPoints[1].class_id == 1)
            {
                goodKeyPoints.push_back(blobKeyPoints[1]);
                goodKeyPoints.push_back(blobKeyPoints[2]);
            }
            else
            {
                goodKeyPoints.push_back(blobKeyPoints[2]);
                goodKeyPoints.push_back(blobKeyPoints[1]);
            }
        }
        else if (blobKeyPoints[1].class_id == 0)
        {
            goodKeyPoints.push_back(blobKeyPoints[1]);
            if (blobKeyPoints[0].class_id == 1)
            {
                goodKeyPoints.push_back(blobKeyPoints[0]);
                goodKeyPoints.push_back(blobKeyPoints[2]);
            }
            else
            {
                goodKeyPoints.push_back(blobKeyPoints[2]);
                goodKeyPoints.push_back(blobKeyPoints[0]);
            }
        }
        else
        {
            goodKeyPoints.push_back(blobKeyPoints[2]);
            if (blobKeyPoints[0].class_id == 1)
            {
                goodKeyPoints.push_back(blobKeyPoints[0]);
                goodKeyPoints.push_back(blobKeyPoints[1]);
            }
            else
            {
                goodKeyPoints.push_back(blobKeyPoints[1]);
                goodKeyPoints.push_back(blobKeyPoints[0]);
            }
        }
        for (int i = 0; i < goodKeyPoints.size(); i++)
        {
            goodPoints.push_back(goodKeyPoints[i].pt);
        }

    }
}

