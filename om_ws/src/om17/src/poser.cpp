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
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include <raspicam/raspicam_cv.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco.hpp>

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

// the displacement of the camera from the center of the robot
const float CAMERA_X = -0.34925f;
const float CAMERA_Y = 0.0f;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    
cv::Ptr<cv::aruco::Board> board =
    cv::aruco::GridBoard::create(5, 3, 0.13, 0.0112522, dictionary);

// the handle to the raspberry pi camera
raspicam::RaspiCam_Cv    camera;

// raspberry pi camera intrinsics and distortion coefficients which are used
// for image correction
cv::Mat                  CAM_INTRINSIC      = cv::Mat::zeros(3, 3, CV_64FC1);
cv::Mat                  CAM_DISTORTION     = cv::Mat::zeros(1, 5, CV_64FC1);

// image received from the raspberry pi camera
cv::Mat                  img;

// 
geometry_msgs::Pose2D    pose;
ros::Publisher           pose_pub;

//
ros::Time old_time;
ros::Duration tick_elapsed;
float dt = 0.0f;
float timer = 0.0f;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(void);
void tick(ros::NodeHandle* nh);
void cleanup(void);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // ROS initialization
    ros::init(argc, argv, "position");
    ros::NodeHandle node_handle;
    
    ros::Rate loop_rate(10);
    
    pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("pose", 10);
    
    init();
    
    // CREATE OPENCV WINDOW
    cv::namedWindow("POSITION", cv::WINDOW_AUTOSIZE);
    
    // sleep for 1 second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // makes sure the initial dt value is zero
    old_time = ros::Time::now();
    
    while (ros::ok())
    {
        // delta time calculation used for stuff
        tick_elapsed = ros::Time::now() - old_time;
        dt = tick_elapsed.toSec();
        old_time = ros::Time::now();
    
        tick(&node_handle);
        
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
    //camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
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
}

void tick(ros::NodeHandle* nh)
{
    timer += dt;
    
    // also check for state here
    if (timer > 1.5f)
    {
        nh->setParam("lost_tracking", true);
    }
    else
    {
        nh->setParam("lost_tracking", false);
    }

    camera.grab();
    camera.retrieve(img);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // look for aruco markers
    cv::aruco::detectMarkers(img, dictionary, corners, ids);

    // at least one marker was found
    if (ids.size() > 0)
    {
    
        // reset the lost-tracking timer
        timer = 0.0f;
    
        cv::aruco::drawDetectedMarkers(img, corners, ids);
        cv::Mat rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(corners, ids, board, CAM_INTRINSIC, CAM_DISTORTION, rvec, tvec);
        
        if (valid > 0)
        {
            cv::aruco::drawAxis(img, CAM_INTRINSIC, CAM_DISTORTION, rvec, tvec, 0.39);

            // translation from camera coords to object coords
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat cameraRotationVector;
            cv::Rodrigues(R.t(), cameraRotationVector);
            cv::Mat cameraTranslationVector = -R.t()*tvec;
    
            // publishing robot location on 
            pose.x = cameraTranslationVector.at<double>(2,0);
            pose.y = cameraTranslationVector.at<double>(0,0);
            pose.theta = -cameraRotationVector.at<double>(2, 0);
 
            pose.x -= CAMERA_X * cos(pose.theta);
            pose.y -= CAMERA_Y * sin(1.57079632679f - pose.theta);

            // in object space, y = 0 means the robot is centered on the field
            // (y = 1.89f)
            pose.y += 1.89 - .2;
    
            // make sure none of our field coordinates are negative
            // (this shouldn't be possible but it's still good to check)
            pose.x = std::max((float)pose.x, 0.0f);
            pose.y = std::max((float)pose.y, 0.0f);
            
            // min for field dimensions
            
            pose_pub.publish(pose);
        }
    }
        
    cv::imshow("POSITION", img);
    cv::waitKey(10);
}

void cleanup(void)
{
    camera.release();
    cv::destroyAllWindows();
}

