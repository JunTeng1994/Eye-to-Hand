#include "stdafx.h"
#include <iostream>
#include <cstdlib>
#include <string>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//int protonect_shutdown;
//void sigint_handler(int s)
//{
//    //protonect_shutdown = true;
//    std::cout << "sinint_handler" << std::endl;
//}
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CKinectWithLibfreenect2
{

    static const int cColorWidth = 1920;
    static const int cColorHeigh = 1080;
    static const int cDepthWidth = 512;
    static const int cDepthHeigh = 424;


public:
    CKinectWithLibfreenect2(std::string str = "");
    ~CKinectWithLibfreenect2();

    bool InitKinectSensor();

    bool OpenKinectDevice();

    bool update();

    cv::Mat rgbImg, irImg;

    cv::Mat cameraMatrix, distCoeffs;

    void getPointCloud(PointCloudT::Ptr& cloud_in);

    void WriteCameraParams(std::string str = "camera_ir.yaml");

    void ReadCameraParams(std::string str = "camera_ir,yaml");

    std::string getDeviceSerial(void);

    int deviceId;

    std::string deviceName;

private:

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device* dev;

    libfreenect2::SyncMultiFrameListener listener;
    libfreenect2::FrameMap frames;

    std::string serial;

    size_t framemax;

    libfreenect2::Registration* registration;
    libfreenect2::Frame undistorted, registered;



};