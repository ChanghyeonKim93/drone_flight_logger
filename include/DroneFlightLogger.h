#ifndef _DRONE_FLIGHT_LOGGER_H_
#define _DRONE_FLIGHT_LOGGER_H_

#include <iostream>
#include <ros/ros.h>
#include <sys/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <ctime>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<double,7,1> PoseVector;
typedef Eigen::Matrix<double,6,1> VelocityVector;

typedef std::string TopicTime;

class DroneFlightLogger {

public:
  DroneFlightLogger(std::string folder_dir);
  ~DroneFlightLogger();
   void current_pose_addline(const PoseVector& current_pose, const TopicTime& curr_time );
   void truth_pose_addline(const PoseVector& truth_pose, const TopicTime& curr_time );
   void desired_addline(const PoseVector& desired_pose,  const TopicTime& curr_time );
   void vo_pose_addline(const PoseVector& vo_pose, const TopicTime& curr_time);
private:
  std::string folder_dir;

  PoseVector current_pose, truth_pose, desired_pose, vo_pose;

  std::ofstream file_current_pose, file_truth_pose, file_desired, file_vo_pose;
 // std::string   file_image_name, file_imu_name, file_pose_name;

};

#endif
