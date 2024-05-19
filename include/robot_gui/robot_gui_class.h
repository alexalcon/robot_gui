#pragma once

#include "ros/subscriber.h"
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"

#include <string>
#include <chrono>

class RobotGUI {
// robot info required data members 
// subscriber callback function 
// methods that make up the GUI application
private:
    // 1. GENERAL ROBOT INFFO AREA
    //--------------------------------------------------------------------------------------------
    // robot info subscriber class members
    //##########################################################################################
    ros::Subscriber robotinfo_sub;
    std::string robotinfo_topic;
    robotinfo_msgs::RobotInfo10Fields robotinfo_data;

    /**
     * @brief Robot info callback function.
     * @param nh Pointer to a ROS node handle------fill it 
     */
    void robotinfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& robotinfo_data);
    //##########################################################################################

    // timeout utilities to update incoming robot info data
    //########################################################################################## 
    std::chrono::steady_clock::time_point last_message_time;
    const std::chrono::seconds MESSAGE_TIMEOUT = std::chrono::seconds(2); // 1 second timeout

    void resetDisplayData();
    //########################################################################################## 
    //--------------------------------------------------------------------------------------------
    
    // 2. TELEOPERATION BUTTONS
    //-------------------------------------------------------------------------
    ros::Publisher velocity_pub;
    geometry_msgs::Twist velocity_data;
    std::string velocity_topic;
    float linear_velocity_step = 0.3;
    float angular_velocity_step = 0.3;
    
    ros::Timer velocity_timer;
    void publishVelocity(const ros::TimerEvent&);  // timer callback function
    //-------------------------------------------------------------------------
    
    // 3. CURRENT VELOCITIES
    ros::Subscriber velocities_sub;
    void velocitiesCallback(const geometry_msgs::Twist::ConstPtr& velocities_data);
    float linear_velocity = 0.0;
    float angular_velocity = 0.0;

    // 4. ROBOT POSITION (ODOMETRY DATA)
    //--------------------------------------------------------------------------
    ros::Subscriber position_sub;
    void positionCallback(const nav_msgs::Odometry::ConstPtr& position_data);
    std::string position_topic;
    float x_position = 0.0;
    float y_position = 0.0;
    float z_position = 0.0;
    //--------------------------------------------------------------------------

    // general GUI data member
    const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";

    // methods to make up the GUI application
    //------------------------------------------------
    void generalInfoArea(cv::Mat& frame);
    void teleoperationButtons(cv::Mat& frame);
    void currentVelocities(cv::Mat& frame);
    void odometryRobotPosition(cv::Mat& frame);
    // void distanceTraveledService(cv::Mat& frame);
    //------------------------------------------------

public:
    // constructor make a doxygen comment 
    RobotGUI();
    
    // make a doxygen comment for GUI window to show robot info data
    void run();
};
