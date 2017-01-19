#include <fstream>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <vector>
#include <thread>

#include <raspicam/raspicam_cv.h>

#include <opencv2/calib3d/calib3d.hpp>

const int POS_BOARD_WIDTH = 9;
const int POS_BOARD_HEIGHT = 6;
const int POS_INTERSECTION_COUNT = POS_BOARD_WIDTH * POS_BOARD_HEIGHT;
const cv::Size POS_BOARD_SIZE(POS_BOARD_WIDTH, POS_BOARD_HEIGHT);

const double SQUARE_SIZE = 0.025;

void sleep_millis(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

int main(int argc, char *argv[])
{
    raspicam::RaspiCam_Cv camera;
    cv::Mat image;
    
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    if (!camera.open())
    {
        std::cerr << "ERROR: COULD NOT OPEN RASPBERRY PI CAMERA" << std::endl;
        return EXIT_FAILURE;
    }
    
    // CREATE CHECKERBOARD
    std::vector<cv::Point3d> object_points;
    for (int i = 0; i < POS_BOARD_HEIGHT; ++i) {
        for (int j = 0; j < POS_BOARD_WIDTH; ++j) {
            object_points.push_back(
                cv::Point3d(double(j * SQUARE_SIZE), double(i * SQUARE_SIZE), 0)    
            );
        }
    }
    
    // CAMERA CALIBRATION
    cv::Mat intrinsics, distortion;
    
    intrinsics = cv::Mat::zeros(3, 3, CV_64FC1);
    intrinsics.at<double>(0, 0) = 630.9776376778118;
    intrinsics.at<double>(0, 1) = 0;
    intrinsics.at<double>(0, 2) = 319.304184394372;
    intrinsics.at<double>(1, 0) = 0;
    intrinsics.at<double>(1, 1) = 629.4177471374746;
    intrinsics.at<double>(1, 2) = 233.4845950219524;
    intrinsics.at<double>(2, 0) = 0;
    intrinsics.at<double>(2, 1) = 0;
    intrinsics.at<double>(2, 2) = 1;
    distortion = cv::Mat::zeros(1, 5, CV_64FC1);
    distortion.at<double>(0, 0) = 0.09046866588331801;
    distortion.at<double>(0, 1) = -0.2441265531225282;
    distortion.at<double>(0, 2) = 0.0005479489398905696;
    distortion.at<double>(0, 2) = -0.002218002368217438;
    distortion.at<double>(0, 4) = 0;
    
    // CREATE OPENCV WINDOW
    cv::namedWindow("POSE", cv::WINDOW_AUTOSIZE);
    
    // sleep for 1 second
    sleep_millis(1000);
    
    int corner_count = 0;
    std::vector<cv::Point2f> corners;
    
    for (;;)
    {
        camera.grab();
        camera.retrieve(image);
        
        bool found = false;
        found = cv::findChessboardCorners(image, POS_BOARD_SIZE, corners, cv::CALIB_CB_FAST_CHECK);
        
        if (found) {
            cv:drawChessboardCorners(image, POS_BOARD_SIZE, cv::Mat(corners), found);
            
            cv::Mat rvec, tvec;
            cv::solvePnP(object_points, corners, intrinsics, distortion, rvec, tvec);
            
            std::cout << "x: " << tvec.at<double>(2, 0) << std::endl;
            std::cout << "y: " << tvec.at<double>(0, 0) << std::endl;
        }
        
        cv::imshow("POSE", image);
        
        int k = cv::waitKey(1) & 0xFF;
        if (k == 27) break;
    }

    camera.release();

    cv::destroyAllWindows();
    
    return EXIT_SUCCESS;
}
