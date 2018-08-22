#include "myKinect.h"
#include "lightSpot.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/ca>

using namespace std;

int main()
{
    CKinectWithLibfreenect2 myKinect("0");
    CLightSpotDetector spotDector("blobParams.yaml");
    //spotDector.WriteBlobDetectorParams("blobParams.yaml");

    //cv::Mat cameraMatrix, distCoeffs;
    //cv::Mat rvec, tvec;
    //cv::stereoCalibrate(spotDector.blobKeyPoints, spotDector.blobKeyPoints, spotDector.blobKeyPoints, cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, spotDector.blobImg.size(), rvec, tvec, rvec, tvec);
    vector<vector<cv::Point2f> > KeyPoints_1;
    vector<vector<cv::Point2f> > KeyPoints_2;
    vector<vector<cv::Point3f> > ObjectPoints;
    vector<cv::Point3f> ObjectPts;
    cv::Point3f ObjectPt;
    ObjectPt.x = 0.30;    ObjectPt.y = 0.40;    ObjectPt.z = 0;
    ObjectPts.push_back(ObjectPt);
    ObjectPt.x = 0.0;    ObjectPt.y = 0.40;    ObjectPt.z = 0;
    ObjectPts.push_back(ObjectPt);
    ObjectPt.x = 0.0;    ObjectPt.y = 0.0;    ObjectPt.z = 0;
    ObjectPts.push_back(ObjectPt);

    //cout << ObjectPts << endl;
    cv::Mat rVec, tVec;
    cv::Mat E, F;


    if (!myKinect.InitKinectSensor())
    {
        cout << "Initialization failed! Exiting..." << endl;
        return -1;
    }
    myKinect.OpenKinectDevice();

    myKinect.deviceName = "kinect2";

    myKinect.WriteCameraParams("./data/camera_ir_" + myKinect.deviceName + ".yaml");
    //myKinect.ReadCameraParams("camera_ir_1.yaml");
    int frameNum = 0;
    int image_index = 0;
    int cloud_index = 0;
    while (myKinect.update())
    {
        //cv::imshow("rgb image", myKinect.rgbImg);
    
        cv::imshow(myKinect.deviceName, myKinect.irImg);

        //cv::Mat detectImg(myKinect.irImg);
        cv::Mat detectImg;
        cv::Mat detectImg_1;
        cv::undistort(myKinect.irImg, detectImg, myKinect.cameraMatrix, myKinect.distCoeffs);
        //cv::Mat detectImg(myKinect.irImg);

        spotDector.DetectBlobKeyPoints(detectImg);
        cv::imshow("blob_image_" + myKinect.deviceName, spotDector.blobImg);
        
        //cv::cvtColor(myKinect.irImg, detectImg, CV_);
        //spotDector.DetectBlobKeyPoints();
        
        char key = cv::waitKey(10);
        if (key == 's')
        {
            frameNum++;
            char buf[10];
            sprintf(buf, "%d", frameNum);
            string str = buf;
            cv::imwrite("./data/image"+to_string(image_index)+".jpg", myKinect.irImg);
            image_index++;
        }

        if (key == 'p')
        {
            cout << "save point cloud......." << endl;
            PointCloudT::Ptr cloudout(new PointCloudT);
            myKinect.getPointCloud(cloudout);
            //pcl::io::savePCDFile("test1.pcd", *cloud);
            int i = 1;
            while ((i < 40) && myKinect.update())
            {
                PointCloudT::Ptr cloud(new PointCloudT);
                PointCloudT::Ptr cloud1(new PointCloudT);

                myKinect.getPointCloud(cloud);
                if ((cloud->size() == cloudout->size()))
                {
                    for (int j = 0; j < cloud->size(); j++)
                    {
                        cloudout->points[j].x += cloud->points[j].x;
                        cloudout->points[j].y += cloud->points[j].y;
                        cloudout->points[j].z += cloud->points[j].z;
                    }
                }
                else
                {
                    cout << "Read Point Cloud Failed. Don't move Kinect." << endl;
                    break;
                }
                i++;
                if (i == 40)
                {
                    for (int j = 0; j < cloudout->size(); j++)
                    {
                        cloudout->points[j].x /= 40;
                        cloudout->points[j].y /= 40;
                        cloudout->points[j].z /= 40;
                    }
                    pcl::io::savePCDFileBinary("./data/cloud"+to_string(cloud_index)+ ".pcd", *cloudout);
                    cloud_index++;
                }
            }
            cout << "save done." << endl;

        }
        if (key == 27)
        {
            cv::FileStorage fs_("./data/stereo_calib.xml", cv::FileStorage::WRITE);
            fs_ << "imagelist" << "[";
            for (int i = 1; i <= frameNum; i++)
            {
                char buf_[10];
                sprintf(buf_, "%d", i);
                string str_ = buf_;
                fs_ << string("left" + str_ + ".jpg");
                //fs_ << "\n";
                fs_ << string("right" + str_ + ".jpg");
                //fs_ << "\n";
            }
            fs_ << "]";
            break;
        }
        if(key == 'e')
        {
            return 0;
        }
    }

	return 0;
}

