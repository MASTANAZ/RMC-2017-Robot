#include <fstream>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <vector>
#include <thread>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <raspicam/raspicam_cv.h>

#include <opencv2/calib3d/calib3d.hpp>

// the dimensions of our positioning board in number of intersections
const int      POS_BOARD_WIDTH        = 9;
const int      POS_BOARD_HEIGHT       = 6;

// precalculated values that help opencv
const int      POS_INTERSECTION_COUNT = POS_BOARD_WIDTH * POS_BOARD_HEIGHT;
const cv::Size POS_BOARD_SIZE(POS_BOARD_WIDTH, POS_BOARD_HEIGHT);

// the size of a square on our positioning board in millimeters
const double   POS_BOARD_SQUARE_SIZE  = 0.1016f;

void sleep_millis(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "position");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(3);
    ros::Publisher pose_publisher = node_handle.advertise<geometry_msgs::Pose2D>("poser", 10);
    
    // raspberry pi camera initialization
    raspicam::RaspiCam_Cv camera;
    
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    if (!camera.open())
    {
        // TODO: change to ROS logging
        std::cerr << "ERROR: COULD NOT OPEN RASPBERRY PI CAMERA" << std::endl;
        return EXIT_FAILURE;
    }
    
    // the raspberry pi camera intrinsic matrix
    cv::Mat CAM_INTRINSIC = cv::Mat::zeros(3, 3, CV_64FC1);

    CAM_INTRINSIC.at<double>(0, 0) = 630.9776376778118f;
    CAM_INTRINSIC.at<double>(0, 1) = 0.0f;
    CAM_INTRINSIC.at<double>(0, 2) = 319.304184394372f;
    CAM_INTRINSIC.at<double>(1, 0) = 0.0f;
    CAM_INTRINSIC.at<double>(1, 1) = 629.4177471374746f;
    CAM_INTRINSIC.at<double>(1, 2) = 233.4845950219524f;
    CAM_INTRINSIC.at<double>(2, 0) = 0.0f;
    CAM_INTRINSIC.at<double>(2, 1) = 0.0f;
    CAM_INTRINSIC.at<double>(2, 2) = 1.0f;

    // the raspberry pi camera distortion coefficients
    cv::Mat CAM_DISTORTION = cv::Mat::zeros(1, 5, CV_64FC1);

    CAM_DISTORTION.at<double>(0, 0) = 0.09046866588331801f;
    CAM_DISTORTION.at<double>(0, 1) = -0.2441265531225282f;
    CAM_DISTORTION.at<double>(0, 2) = 0.0005479489398905696f;
    CAM_DISTORTION.at<double>(0, 2) = -0.002218002368217438f;
    CAM_DISTORTION.at<double>(0, 4) = 0.0f;
    
    // create vector of points on the checkerboard
    std::vector<cv::Point3d> obj_points;
    
    for (int i = 0; i < POS_BOARD_HEIGHT; ++i) {
        for (int j = 0; j < POS_BOARD_WIDTH; ++j) {
            obj_points.push_back(
                cv::Point3d(
                    double(j * POS_BOARD_SQUARE_SIZE),
                    double(i * POS_BOARD_SQUARE_SIZE),
                    0.0f
                )    
            );
        }
    }
    
    // CREATE OPENCV WINDOW
    cv::namedWindow("POSITION", cv::WINDOW_AUTOSIZE);
    
    // sleep for 1 second
    sleep_millis(1000);
    
    int intersection_count = 0;
    std::vector<cv::Point2f> intersections;
    
    cv::Mat img;
    geometry_msgs::Pose2D pose;
    
    while (ros::ok())
    {
        camera.grab();
        camera.retrieve(img);
        
        bool found = false;
        
        found = cv::findChessboardCorners(
            img,
            POS_BOARD_SIZE,
            intersections,
            cv::CALIB_CB_FAST_CHECK
        );
        
        if (found) {
            cv:drawChessboardCorners(img, POS_BOARD_SIZE, cv::Mat(intersections), found);
            
            // rotation and translation vectors
            cv::Mat rvec, tvec;
            cv::solvePnP(obj_points, intersections, CAM_INTRINSIC, CAM_DISTORTION, rvec, tvec);
            
            std::cout << "x: " << tvec.at<double>(2, 0) << std::endl;
            std::cout << "y: " << tvec.at<double>(0, 0) << std::endl;
            
            pose.x = tvec.at<double>(2, 0);
            pose.y = tvec.at<double>(0, 0);
            
            pose_publisher.publish(pose);
        }
        
        cv::imshow("POSITION", img);
        cv::waitKey(10);
        
        ros::spinOnce();
        
        loop_rate.sleep();
    }

    camera.release();

    cv::destroyAllWindows();
    
    return EXIT_SUCCESS;
}
