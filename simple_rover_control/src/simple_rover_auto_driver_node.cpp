
// ROS includes
#include <ros/ros.h>
#include <std_msgs/Float32.h>



int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_rover_auto_driver");
  ros::NodeHandle nh("~");
  // Different publisher for each wheel ?
  // Or general class for wheel control?
  // Use yaml file?

  // Float messages for control
  std::string rightBackTopic = "/rover/wheel_right_back_controller/command";
  std::string rightFrontTopic = "/rover/wheel_right_front_controller/command";
  std::string leftBackTopic = "/rover/wheel_left_back_controller/command";
  std::string leftFrontTopic = "/rover/wheel_left_front_controller/command";
  ros::Publisher pubRB = nh.advertise<std_msgs::Float32>(rightBackTopic, 1); 
  ros::Publisher pubRF = nh.advertise<std_msgs::Float32>(rightFrontTopic, 1); 
  ros::Publisher pubLB = nh.advertise<std_msgs::Float32>(leftBackTopic, 1); 
  ros::Publisher pubLF = nh.advertise<std_msgs::Float32>(leftFrontTopic, 1); 
  
  ros::Rate rate(30); // Rate in Hz
  while (nh.ok()) {
    std_msgs::Float32 driveMsg;
    driveMsg.data = 100;
    pubRB.publish(driveMsg);
    pubLB.publish(driveMsg);

    rate.sleep();
  }
  return 0;
}