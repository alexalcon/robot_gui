#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"

#include <string>
#include <chrono>

class RobotGUI {
// robot info required data members 
// subscriber callback function 
// methods that make up the GUI application
private:
    // general robot info area declaration
    //----------------------------------------------------------------------------------------------
    // robot info subscriber class members
    //##########################################################################################
    ros::Subscriber robotinfo_sub;
    std::string robotinfo_topic;
    robotinfo_msgs::RobotInfo10Fields robotinfo_data;

    /**
     * @brief Robot info callback function.
     * @param nh Pointer to a ROS node handle------fill it .
     */
    void robotinfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& robotinfo_data);
    //##########################################################################################

    // timeout utilities to update incoming robot info data
    //########################################################################################## 
    std::chrono::steady_clock::time_point last_message_time;
    const std::chrono::seconds MESSAGE_TIMEOUT = std::chrono::seconds(1); // 1 second timeout

    void resetDisplayData();
    //########################################################################################## 
    //----------------------------------------------------------------------------------------------
    
    // general GUI data member
    const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";

    // methods to make up the GUI application
    //------------------------------------------------
    void generalInfoArea(cv::Mat& frame);
    // void teleoperationButtons(cv::Mat& frame);
    // void currentVelocities(cv::Mat& frame);
    // void odometryRobotPosition(cv::Mat& frame);
    // void distanceTraveledService(cv::Mat& frame);
    //------------------------------------------------

public:
    // constructor make a doxygen comment 
    RobotGUI();
    
    // make a doxygen comment for GUI window to show robot info data
    void run();
};
