#include <DroneFlightLogger.h>

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

DroneFlightLogger::DroneFlightLogger(std::string folder_dir_){
	std::string folder_create_command, file_name, folder_dir_temp;
	this->folder_dir = folder_dir_;

  	// folder_dir_temp = this->folder_dir;

	// if(strcmp(*folder_dir_temp.end(),'/')!=0) folder_dir_temp+='/';

	folder_create_command = "mkdir " + this->folder_dir;
	system(folder_create_command.c_str());

	file_name = this->folder_dir+"current.txt";
	this->file_current_pose.open(file_name.c_str());
	this->file_current_pose << "# time tx ty tz qx qy qz qw\n";

	file_name = this->folder_dir+"truth.txt";
	this->file_truth_pose.open(file_name.c_str());
	this->file_truth_pose << "# time tx ty tz qx qy qz qw\n";

	file_name = this->folder_dir+"desired.txt";
	this->file_desired.open(file_name.c_str());
	this->file_desired << "# time tx ty tz qx qy qz qw\n";

	file_name = this->folder_dir+"vo.txt";
	this->file_vo_pose.open(file_name.c_str());
	this->file_vo_pose << "# time tx ty tz qx qy qz qw\n";
};

DroneFlightLogger::~DroneFlightLogger(){
	this->file_current_pose.close();
	this->file_truth_pose.close();
	this->file_desired.close();
	this->file_vo_pose.close();
	ROS_INFO_STREAM("Drone flight logger is terminated.\n");
};

void DroneFlightLogger::current_pose_addline(const PoseVector& current_pose_, const TopicTime& curr_time ){

	file_current_pose <<curr_time<< "\t";
	file_current_pose <<std::setprecision(13);
	file_current_pose.unsetf(std::ios::fixed);
	for(int i=0; i<7; i++)
	{
		file_current_pose << current_pose_(i,0) << "\t";
	}
	file_current_pose << "\n";
};

void DroneFlightLogger::truth_pose_addline(const PoseVector& truth_pose_, const TopicTime& curr_time ){

	file_truth_pose <<curr_time<< "\t";
	file_truth_pose <<std::setprecision(13);
	file_truth_pose.unsetf(std::ios::fixed);
	for(int i=0; i<7; i++)
	{
		file_truth_pose << truth_pose_(i,0) << "\t";
	}
	file_truth_pose << "\n";
};


void DroneFlightLogger::desired_addline(const PoseVector& desired_pose_, const TopicTime& curr_time ){

	file_desired <<curr_time<< "\t";
	file_desired <<std::setprecision(13);
	file_desired.unsetf(std::ios::fixed);
	for(int i=0; i<7; i++)
	{
		file_desired << desired_pose_(i,0) << "\t";
	}
	file_desired << "\n";
};

void DroneFlightLogger::vo_pose_addline(const PoseVector& vo_pose_, const TopicTime& curr_time ){

	file_vo_pose <<curr_time<< "\t";
	file_vo_pose <<std::setprecision(13);
	file_vo_pose.unsetf(std::ios::fixed);
	for(int i=0; i<7; i++)
	{
		file_vo_pose << vo_pose_(i,0) << "\t";
	}
	file_vo_pose << "\n";
};
