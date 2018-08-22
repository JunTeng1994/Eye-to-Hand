#include "myKinect.h"



//typedef void(CKinectWithLibfreenect2::sigint_handler){};


bool CKinectWithLibfreenect2::InitKinectSensor()
{
    /*if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return false;
    }*/
    if (serial == "")
    {
        if (freenect2.enumerateDevices() == 0)
        {
            std::cout << "no device connected!" << std::endl;
            return false;
        }
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    else if (serial == "0")
    {
        if (freenect2.enumerateDevices() == 0)
        {
            std::cout << "no device connected!" << std::endl;
            return false;
        }
        serial = freenect2.getDeviceSerialNumber(0);
    }
    else if (serial == "1")
    {
        serial = freenect2.getDeviceSerialNumber(1);
    }

    std::cout << "serial: " << serial << std::endl;

    //dev = freenect2.openDevice(serial);
    ////signal(SIGINT, sigint_handler);
    ////protonect_shutdown = false;

    //if (dev == 0)
    //{
    //    std::cout << "Failure opening device!" << std::endl;
    //    return false;
    //}

    ////int types = 0;
    ////types |= libfreenect2::Frame::Color;
    ////types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    //dev->setColorFrameListener(&listener);
    //dev->setIrAndDepthFrameListener(&listener);

    //if (!dev->start())
    //    return false;

    //std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    //std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    ////registration->undistortDepth                                                
    //
    //registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    return true;
}

bool CKinectWithLibfreenect2::OpenKinectDevice()
{
    dev = freenect2.openDevice(serial);
    //signal(SIGINT, sigint_handler);
    //protonect_shutdown = false;

    if (dev == 0)
    {
        std::cout << "Failure opening device!" << std::endl;
        return false;
    }

    //int types = 0;
    //types |= libfreenect2::Frame::Color;
    //types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->start())
        return false;

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    //registration->undistortDepth                                                
    
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    return true;
}

CKinectWithLibfreenect2::CKinectWithLibfreenect2(std::string str) :listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
undistorted(cDepthWidth, cDepthHeigh, 4),
registered(cDepthWidth, cDepthHeigh, 4),
dev(0)
{
    deviceName = "";
    deviceId = 0;
    serial = str;
}

bool CKinectWithLibfreenect2::update()
{
    //if (!protonect_shutdown)

    if (!listener.waitForNewFrame(frames, 10 * 1000))//10 secs
    {
        std::cout << "timeout! Exiting..." << std::endl;
        return false;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply(rgb, depth, &undistorted, &registered);

    cv::Mat rgb_mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::Mat ir_mat(ir->height, ir->width, CV_32FC1, ir->data);
    ir_mat /= 65535;
    ir_mat.convertTo(irImg, CV_8UC1, 255);
    //cv::resize(ir_mat, irImg, cv::Size(cDepthWidth, cDepthHeigh));
    cv::resize(rgb_mat, rgbImg, cv::Size(cColorWidth, cColorHeigh));
    cv::flip(rgbImg, rgbImg, 1);
    cv::flip(irImg, irImg, 1);
    //rgbImg = rgb_mat;
    //irImg = ir_mat;
    //cv::imshow("test", rgb_mat);
    //cv::waitKey(0); 
    listener.release(frames);
    return true;
}

CKinectWithLibfreenect2::~CKinectWithLibfreenect2()
{
    dev->stop();
    dev->close();
    delete registration;
}

void CKinectWithLibfreenect2::getPointCloud(PointCloudT::Ptr& cloud_in)
{

    float x = 0, y = 0, z = 0;
    float rgb = 0;
    PointT pt;
    for (int i = 0; i < cDepthHeigh; i++)
    {
        for (int j = 0; j < cDepthWidth; j++)
        {
            registration->getPointXYZRGB(&undistorted, &registered, i, j, x, y, z, rgb);
            pt.x = x;
            pt.y = -y;
            pt.z = z;
            const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
            pt.b = p[0];
            pt.g = p[1];
            pt.r = p[2];
            cloud_in->push_back(pt);
        }
    }
}

void CKinectWithLibfreenect2::WriteCameraParams(std::string str)
{
    if (dev == 0)
    {
        std::cout << "No device found." << std::endl;
    }
    else
    {
        libfreenect2::Freenect2Device::IrCameraParams ir_params;
        //cv::Mat cameraMatrix_, distCoeffs_;
        cv::Mat cameraMatrix_(cv::Size(3, 3), CV_32FC1, cv::Scalar(0));
        cv::Mat distCoeffs_(cv::Size(1, 5), CV_32FC1, cv::Scalar(0));
        ir_params = dev->getIrCameraParams();
        cameraMatrix_.at<float>(0, 0) = ir_params.fx;
        cameraMatrix_.at<float>(0, 2) = cDepthWidth - ir_params.cx;
        cameraMatrix_.at<float>(1, 1) = ir_params.fy;
        cameraMatrix_.at<float>(1, 2) = ir_params.cy;
        cameraMatrix_.at<float>(2, 2) = 1;
        distCoeffs_.at<float>(0, 0) = ir_params.k1;
        distCoeffs_.at<float>(0, 1) = ir_params.k2;
        distCoeffs_.at<float>(0, 2) = ir_params.p1;
        distCoeffs_.at<float>(0, 3) = ir_params.p2;
        distCoeffs_.at<float>(0, 4) = ir_params.k2;

        cv::FileStorage fs(str, cv::FileStorage::WRITE);
        fs << "cameraMatrix" << cameraMatrix_;
        fs << "distortionCoefficients" << distCoeffs_;
        fs.release();
        cv::resize(cameraMatrix_, cameraMatrix, cameraMatrix_.size());
        cv::resize(distCoeffs_, distCoeffs, distCoeffs_.size());
        std::cout << "cameraMatrix: " << std::endl << cameraMatrix << std::endl
            << "distortionCoefficients: " << std::endl << distCoeffs << std::endl;

    }
}

void CKinectWithLibfreenect2::ReadCameraParams(std::string str)
{
    cv::FileStorage fs(str, cv::FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distortionCoefficients"] >> distCoeffs;
    std::cout << "cameraMatrix: " << std::endl << cameraMatrix << std::endl
        << "distortionCoefficients: " << std::endl << distCoeffs << std::endl;
    fs.release();
}

std::string CKinectWithLibfreenect2::getDeviceSerial(void)
{
    std::string str(this->serial);
    return str;
}
