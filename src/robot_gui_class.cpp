#include "robot_gui/robot_gui_class.h"
#include "nav_msgs/Odometry.h"

// constructor
RobotGUI::RobotGUI() {
    ros::NodeHandle nh;

    // general robot info area members initialization
    robotinfo_topic = "robot_info"; // topic name for general robot info area  
    robotinfo_sub = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(this->robotinfo_topic, 1000, 
                                                                    &RobotGUI::robotinfoCallback, this);
    
    // teleoperation buttons members initialization
    velocity_topic = "cmd_vel"; // topic name for velocity teleoperation buttons
    velocity_pub = nh.advertise<geometry_msgs::Twist>(this->velocity_topic, 10);
    velocity_timer = nh.createTimer(ros::Duration(0.1), &RobotGUI::publishVelocity, this);

    // current velocities members initialization
    velocities_sub = nh.subscribe<geometry_msgs::Twist>(this->velocity_topic, 1000, 
                                                 &RobotGUI::velocitiesCallback, this);

    // robot's position (odometry data) initialization
    position_topic = "odom";
    position_sub = nh.subscribe<nav_msgs::Odometry>(this->position_topic, 1000, 
                                                    &RobotGUI::positionCallback, this);
}

// methods to make up the GUI app
//---------------------------------------------------------------------------------------
// 1. GENERAL ROBOT INFFO AREA
// display each data field from the message
void RobotGUI::generalInfoArea(cv::Mat& frame) {
    // create window at (10, 10) with size 380x203 (width x height) and title
    cvui::window(frame, 10, 10, 380, 203, "Topic: " + this->robotinfo_topic);

    // starting position for displaying text
    int y = 35;
    int dy = 18; // vertical spacing between lines

    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 01: %s", 
                 this->robotinfo_data.data_field_01.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 02: %s", 
                 this->robotinfo_data.data_field_02.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 03: %s", 
                 this->robotinfo_data.data_field_03.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 04: %s", 
                 this->robotinfo_data.data_field_04.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 05: %s", 
                 this->robotinfo_data.data_field_05.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 06: %s", 
                 this->robotinfo_data.data_field_06.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 07: %s", 
                 this->robotinfo_data.data_field_07.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 08: %s", 
                 this->robotinfo_data.data_field_08.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 09: %s", 
                 this->robotinfo_data.data_field_09.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 10: %s", 
                 this->robotinfo_data.data_field_10.c_str());
    y += dy;
}

// 2. TELEOPERATION BUTTONS
// velocity teleoperation buttons
void RobotGUI::teleoperationButtons(cv::Mat& frame) {
    int infoHeight = 180;               // same as used in generalInfoArea
    int gap = 35;                       // gap size between info area and buttons
    int baseY = infoHeight + gap + 10;  // adjust baseY to account for the gap

    int buttonHeight = 60;
    int buttonWidth = 120;

    // calculate center positions based on the frame width
    int centerX = (frame.cols - buttonWidth) / 2;

    // vevlocity buttons logic
    //-------------------------------------------------------
    // forward button at the top
    if (cvui::button(frame, 
                     centerX, 
                     baseY, 
                     buttonWidth, 
                     buttonHeight, 
                     "Forward")) {
        
        // the button was clicked, update the Twist message
        velocity_data.linear.x += linear_velocity_step;
    }

    // middle row buttons
    if (cvui::button(frame, 
                     centerX - buttonWidth - 10, 
                     baseY + buttonHeight + 10, 
                     buttonWidth, 
                     buttonHeight, 
                     "Left")) {
        // the button was clicked, update the Twist message
        velocity_data.angular.z += angular_velocity_step;
    }
    if (cvui::button(frame, 
                     centerX, 
                     baseY + buttonHeight + 10, 
                     buttonWidth, 
                     buttonHeight, 
                     "Stop")) {
        // the button was clicked, update the Twist message
        velocity_data.linear.x = 0.0;
        velocity_data.angular.z = 0.0;
    }
    if (cvui::button(frame, 
                     centerX + buttonWidth + 10, 
                     baseY + buttonHeight + 10, 
                     buttonWidth, 
                     buttonHeight, 
                     "Right")) {
        // the button was clicked, update the Twist message
        velocity_data.angular.z -= angular_velocity_step;     
    }

    // backward button at the bottom
    if (cvui::button(frame, 
                     centerX, 
                     baseY + 2 * (buttonHeight + 10), 
                     buttonWidth, buttonHeight, 
                     "Backward")) {
        // the button was clicked, update the Twist message
        velocity_data.linear.x -= linear_velocity_step;
    }
    //-------------------------------------------------------
}
// 3. CURRENT VELOCITIES
// display current velocities in small GUI windows
void RobotGUI::currentVelocities(cv::Mat& frame) {
    // define positions for the windows
    int baseY = 435; // this should be adjusted based on your GUI layout needs
    int windowWidth = 185;
    int windowHeight = 40;
    int padding = 10;  // space between windows

    // linear velocity window
    cvui::window(frame, 10, baseY, windowWidth, windowHeight, "Linear Velocity");
    cvui::printf(frame, 15, baseY + 24, 0.40, 0x33ffff, 
                 "Linear velocity: %0.2f m/s", this->linear_velocity);

    // angular velocity window
    cvui::window(frame, 10 + windowWidth + padding, baseY, windowWidth, 
                 windowHeight, "Angular Velocity");
    cvui::printf(frame, 15 + windowWidth + padding, baseY + 24, 0.40, 0x33ffff,     
                 "Angular velocity: %0.2f rad/s", this->angular_velocity);
}

// 4. ROBOT POSITION (ODOMETRY DATA)
// display current robot's position
void RobotGUI::odometryRobotPosition(cv::Mat& frame) {
    // Set up the window location and size
    int posX = 10;  // Horizontal position of the window on the frame
    int posY = 500;  // Vertical position from the top
    int width = 380; // Width of the window
    int height = 87; // Height of the window

    // Create the window for displaying odometry data
    cvui::window(frame, posX, posY, width, height, 
                 "Estimated robot position based off odometry");

    // Display the X, Y, Z coordinates in the window
    int startX = posX + 15;  // Start position for text, slightly indented
    int startY = posY + 25;  // Start position for text, vertically adjusted
    int lineSpacing = 20;    // Space between lines of text

    cvui::printf(frame, startX, startY, 0.6, 0xffffff, 
                 "X: %0.17f", this->x_position);
    cvui::printf(frame, startX, startY + lineSpacing, 0.6, 0xffffff, 
                 "Y: %0.17f", this->y_position);
    cvui::printf(frame, startX, startY + 2 * lineSpacing, 0.6, 0xffffff, 
                 "Z: %0.17f", this->z_position); 
}
//---------------------------------------------------------------------------------------

// additional utilities for the GUI app
//--------------------------------------------------------------------------------------------------------
// 1. GENERAL ROBOT INFFO AREA
//######################################################################################################
// timeout utility for robot info subscriber (general info area)
void RobotGUI::resetDisplayData() {
    this->robotinfo_data = robotinfo_msgs::RobotInfo10Fields(); // reset to default constructed state
}

// subscriber callback function for general robot info area
void RobotGUI::robotinfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& robotinfo_data) {
    this->robotinfo_data = *robotinfo_data;
    last_message_time = std::chrono::steady_clock::now(); // update the last message time
    ROS_DEBUG("Robot info data updated.");
}
//######################################################################################################

// 2. TELEOPERATION BUTTONS
// timer callback to publish velocity (teleoperation buttons)
void RobotGUI::publishVelocity(const ros::TimerEvent&) {
    velocity_pub.publish(velocity_data);
}

// 3. CURENT VELOCITIES
// subscriber callback function for current velocities
void RobotGUI::velocitiesCallback(const geometry_msgs::Twist::ConstPtr &velocities_data) {
    this->linear_velocity = velocities_data->linear.x;
    this->angular_velocity = velocities_data->angular.z;
    ROS_DEBUG("Robot current velocities data updated.");
}

// 4. ROBOT POSITION (ODOMETRY DATA)
void RobotGUI::positionCallback(const nav_msgs::Odometry::ConstPtr& position_data) {
    this->x_position = position_data->pose.pose.position.x;
    this->y_position = position_data->pose.pose.position.y;
    this->z_position = position_data->pose.pose.position.z;
    ROS_DEBUG("Robot current position data updated.");
}
//--------------------------------------------------------------------------------------------------------

// GUI app window (main functionality)
void RobotGUI::run() {
  // this line initializes a cv::Mat object called frame 
  // that represents an image of 1000 pixels in height and 
  // 600 pixels in width, with three color channels (RGB) 
  cv::Mat frame = cv::Mat(1000, 400, CV_8UC3);

  // init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // timeout config
    auto now = std::chrono::steady_clock::now();
    if (now - last_message_time > MESSAGE_TIMEOUT) {
        resetDisplayData(); // Reset data if timeout has passed
    }

    // fill the frame with a nice color
    frame = cv::Scalar(51, 0, 51);

    // calling GUI methods functionalities  
    generalInfoArea(frame);
    teleoperationButtons(frame);
    currentVelocities(frame);
    odometryRobotPosition(frame);
    
    // update cvui internal stuff
    cvui::update();

    // show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    // spin as a single-threaded node
    ros::spinOnce();
  }
}