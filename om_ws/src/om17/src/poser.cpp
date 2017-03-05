// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//
// DESCRIPTION:
//

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <vector>
#include <thread>
#include <algorithm>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <raspicam/raspicam_cv.h>

#include <opencv2/calib3d/calib3d.hpp>

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// the dimensions of our positioning board in number of intersections
const int      POS_BOARD_WIDTH        = 9;
const int      POS_BOARD_HEIGHT       = 6;

// precalculated values that help opencv
const int      POS_INTERSECTION_COUNT = POS_BOARD_WIDTH * POS_BOARD_HEIGHT;
const cv::Size POS_BOARD_SIZE(POS_BOARD_WIDTH, POS_BOARD_HEIGHT);

// the size of a square on our positioning board in millimeters
const double   POS_BOARD_SQUARE_SIZE  = 0.1016f;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

// the handle to the raspberry pi camera
raspicam::RaspiCam_Cv    camera;

// raspberry pi camera intrinsics and distortion coefficients which are used
// for image correction
cv::Mat                  CAM_INTRINSIC      = cv::Mat::zeros(3, 3, CV_64FC1);
cv::Mat                  CAM_DISTORTION     = cv::Mat::zeros(1, 5, CV_64FC1);

//
std::vector<cv::Point3d> object_points;

//
int                      intersection_count = 0;
std::vector<cv::Point2f> intersections;

// image received from the raspberry pi camera
cv::Mat                  img;

// 
geometry_msgs::Pose2D    pose;
ros::Publisher           pose_pub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(void);
void tick(void);
void cleanup(void);

void sleep_millis(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "position");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(5);
    pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("pose", 10);
    
    init();
    
    // CREATE OPENCV WINDOW
    cv::namedWindow("POSITION", cv::WINDOW_AUTOSIZE);
    
    // sleep for 1 second
    sleep_millis(1000);
    
    while (ros::ok())
    {
        tick();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    cleanup();
    
    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void init(void)
{
    // raspberry pi camera properties
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    // make sure the camera was openend properly
    if (!camera.open())
    {
        ROS_FATAL("COULD NOT OPEN RASPBERRY PI CAMERA");
        std::exit(EXIT_FAILURE);
    }

    // these values were found with a camera calibrator program
    CAM_INTRINSIC.at<double>(0, 0) = 630.9776376778118f;
    CAM_INTRINSIC.at<double>(0, 1) = 0.0f;
    CAM_INTRINSIC.at<double>(0, 2) = 319.304184394372f;
    CAM_INTRINSIC.at<double>(1, 0) = 0.0f;
    CAM_INTRINSIC.at<double>(1, 1) = 629.4177471374746f;
    CAM_INTRINSIC.at<double>(1, 2) = 233.4845950219524f;
    CAM_INTRINSIC.at<double>(2, 0) = 0.0f;
    CAM_INTRINSIC.at<double>(2, 1) = 0.0f;
    CAM_INTRINSIC.at<double>(2, 2) = 1.0f;

    // these values were found with a camera calibrator program
    CAM_DISTORTION.at<double>(0, 0) = 0.09046866588331801f;
    CAM_DISTORTION.at<double>(0, 1) = -0.2441265531225282f;
    CAM_DISTORTION.at<double>(0, 2) = 0.0005479489398905696f;
    CAM_DISTORTION.at<double>(0, 2) = -0.002218002368217438f;
    CAM_DISTORTION.at<double>(0, 4) = 0.0f;
    
    // create the vector of points on the checkerboard
    for (int i = 0; i < POS_BOARD_HEIGHT; ++i)
    {
        for (int j = 0; j < POS_BOARD_WIDTH; ++j)
        {
            object_points.push_back
            (
                cv::Point3d
                (
                    double(j * POS_BOARD_SQUARE_SIZE),
                    double(i * POS_BOARD_SQUARE_SIZE),
                    0.0f
                )
            );
        }
    }
}

void tick(void)
{
    camera.grab();
    camera.retrieve(img);
    
    bool found = false;
        
    found = cv::findChessboardCorners
    (
        img,
        POS_BOARD_SIZE,
        intersections,
        cv::CALIB_CB_FAST_CHECK
    );
    
    if (found)
    {
        cv:drawChessboardCorners(img, POS_BOARD_SIZE, cv::Mat(intersections), found);
            
        // rotation and translation vectors
        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, intersections, CAM_INTRINSIC, CAM_DISTORTION, rvec, tvec);
            
        pose.x = tvec.at<double>(2, 0);
        pose.y = tvec.at<double>(0, 0);
            
        // translation from camera coords to object coords
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat cameraRotationVector;
        cv::Rodrigues(R.t(), cameraRotationVector);
        cv::Mat cameraTranslationVector = R.t()*tvec;

        // publishing robot location on 
        pose.x = cameraTranslationVector.at<double>(2,0);
        pose.y = cameraTranslationVector.at<double>(0,0);
        pose.theta = cameraRotationVector.at<double>(1, 0);

        // in object space, y = 0 means the robot is centered on the field
        // (y = 1.89f)
        pose.y += 1.89f;

        // rotate the robot's angle by 360 degrees so we don't have to
        // transmit negative data over the network
        pose.theta += 2.0f * 3.14159f;

        // make sure none of our field coordinates are negative
        // (this shouldn't be possible but it's still good to check)
        pose.x = std::max((float)pose.x, 0.0f);
        pose.y = std::max((float)pose.y, 0.0f);
        
        pose_pub.publish(pose);
    }
        
    cv::imshow("POSITION", img);
    cv::waitKey(10);
}

void cleanup(void)
{
    camera.release();
    cv::destroyAllWindows();
}

