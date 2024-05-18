#include "robot_gui/robot_gui_class.h"

// constructor
RobotGUI::RobotGUI() {
    ros::NodeHandle nh;

    robotinfo_topic = "robot_info";
    robotinfo_sub = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(this->robotinfo_topic, 1000, 
                                                                    &RobotGUI::robotinfoCallback, this);
}

// general robot info area members 
//-----------------------------------------------------------------------------------------------------
// subscriber callback function for general robot info area
void RobotGUI::robotinfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& robotinfo_data) {
    this->robotinfo_data = *robotinfo_data;
    last_message_time = std::chrono::steady_clock::now(); // update the last message time
    ROS_DEBUG("Robot info data updated.");
}
//-----------------------------------------------------------------------------------------------------

// methods to make up the GUI app
//---------------------------------------------------------------------------------------------------------------
// display each data field from the message
void RobotGUI::generalInfoArea(cv::Mat& frame) {
    // create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 10, 10, 380, 203, "Topic: " + this->robotinfo_topic);

    // starting position for displaying text
    int y = 35;
    int dy = 18; // vertical spacing between lines

    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 01: %s", this->robotinfo_data.data_field_01.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 02: %s", this->robotinfo_data.data_field_02.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 03: %s", this->robotinfo_data.data_field_03.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 04: %s", this->robotinfo_data.data_field_04.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 05: %s", this->robotinfo_data.data_field_05.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 06: %s", this->robotinfo_data.data_field_06.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 07: %s", this->robotinfo_data.data_field_07.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 08: %s", this->robotinfo_data.data_field_08.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 09: %s", this->robotinfo_data.data_field_09.c_str());
    y += dy;
    cvui::printf(frame, 15, y, 0.4, 0x80ff00, "Data Field 10: %s", this->robotinfo_data.data_field_10.c_str());
    y += dy;
}


//---------------------------------------------------------------------------------------------------------------

// additional utilities for the GUI app
//-----------------------------------------------------------------------------------------------------
// timeout utility for robot info subscriber (general info area)
void RobotGUI::resetDisplayData() {
    this->robotinfo_data = robotinfo_msgs::RobotInfo10Fields(); // reset to default constructed state
}
//-----------------------------------------------------------------------------------------------------

// GUI app window (main method functionality)
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