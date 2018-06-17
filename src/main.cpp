#include <ros/ros.h>

#include <DroneFlightLogger.h>

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseStamped.h> // for current position
#include <geometry_msgs/TransformStamped.h> // for vicon 
#include <mavros_msgs/PositionTarget.h> // for desired position

bool current_pose_updated  = false;
bool truth_pose_updated  = false;
bool desired_updated  = false;
bool vo_pose_updated  = false;

typedef Eigen::Matrix<double,7,1> PoseVector;
typedef Eigen::Matrix<double,6,1> VelocityVector;

typedef std::string TopicTime;

PoseVector current_pose = PoseVector::Zero();
PoseVector truth_pose   = PoseVector::Zero();
PoseVector desired_pose = PoseVector::Zero();
PoseVector vo_pose      = PoseVector::Zero();

TopicTime current_pose_time, truth_pose_time, desired_time, vo_pose_time;


std::string dtos(double x){
	std::stringstream s;
	s<<std::setprecision(6) << std::fixed << x;
	return s.str();
}

void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_pose(0,0) = msg->pose.position.x;
	current_pose(1,0) = msg->pose.position.y;
	current_pose(2,0) = msg->pose.position.z;
	//current_pose(3,0) = msg->pose.orientation.x;
	//current_pose(4,0) = msg->pose.orientation.y;
	//current_pose(5,0) = msg->pose.orientation.z;
	//current_pose(6,0) = msg->pose.orientation.w;
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	current_pose_time = dtos(curr_time);
	ROS_INFO_STREAM("current pose updated.");
	current_pose_updated = true;
}


void vo_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	vo_pose(0,0) = msg->pose.position.x;
	vo_pose(1,0) = msg->pose.position.y;
	vo_pose(2,0) = msg->pose.position.z;
	
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	vo_pose_time = dtos(curr_time);
	ROS_INFO_STREAM("current pose updated.");
	vo_pose_updated = true;
}

void truth_pose_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
	truth_pose(0,0) = msg->transform.translation.x;
	truth_pose(1,0) = msg->transform.translation.y;
	truth_pose(2,0) = msg->transform.translation.z;
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	truth_pose_time = dtos(curr_time);
	ROS_INFO_STREAM("truth pose updated.");
	truth_pose_updated = true;
}

void desired_callback(const mavros_msgs::PositionTarget::ConstPtr& msg){
	desired_pose(0,0) = msg->position.x;
	desired_pose(1,0) = msg->position.y;
	desired_pose(2,0) = msg->position.z;
	//desired_pose(3,0) = msg->pose.orientation.x;
	//desired_pose(4,0) = msg->pose.orientation.y;
	//desired_pose(5,0) = msg->pose.orientation.z;
	//desired_pose(6,0) = msg->pose.orientation.w;
	double curr_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	desired_time = dtos(curr_time);
	ROS_INFO_STREAM("desired updated.");
	desired_updated = true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_flight_logger_node");
	ros::NodeHandle nh("~");

	std::string folder_dir;
	if(ros::param::get("~folder_dir", folder_dir)==false){
		ROS_ERROR_STREAM("Cannot make the directory !\n");
	}
	std::cout<<"Folder directory : " <<folder_dir<<std::endl;
	DroneFlightLogger *drone_flight_logger = new DroneFlightLogger(folder_dir);

	ros::Subscriber current_pose_subsc, truth_pose_subsc, desired_subsc, vo_pose_subsc;

    	std::string current_pose_topic_name = std::string("/measurement_selection/pose");
	std::string truth_pose_topic_name   = std::string("/vicon/CHK_M100/CHK_M100");
	std::string desired_topic_name      = std::string("/gcs/setpoint_raw/position");
	std::string vo_pose_topic_name      = std::string("/sgpvo/pose");

    	ros::param::get("~current_pose_topic_name", current_pose_topic_name);
    	ros::param::get("~truth_pose_topic_name",   truth_pose_topic_name);
    	ros::param::get("~desired_topic_name",      desired_topic_name);
    	ros::param::get("~vo_pose_topic_name",   vo_pose_topic_name);


	current_pose_subsc  = nh.subscribe<geometry_msgs::PoseStamped>(current_pose_topic_name, 5, &current_pose_callback);
	truth_pose_subsc    = nh.subscribe<geometry_msgs::TransformStamped>(truth_pose_topic_name, 5, &truth_pose_callback);
	desired_subsc       = nh.subscribe<mavros_msgs::PositionTarget>(desired_topic_name, 5, &desired_callback);

	vo_pose_subsc	    = nh.subscribe<geometry_msgs::PoseStamped>(vo_pose_topic_name, 5, &vo_pose_callback);
	while(ros::ok()){
		ros::spinOnce(); // so fast
		if(current_pose_updated==true){
			ros::Time time = ros::Time::now();
			double curr_time = (double)(time.sec*1e6+time.nsec/1000)/1000000.0;
			current_pose_time = dtos(curr_time);

			drone_flight_logger->current_pose_addline(current_pose, current_pose_time);
			current_pose_updated = false;
		}
		if(truth_pose_updated==true){
			ros::Time time = ros::Time::now();
			double curr_time = (double)(time.sec*1e6+time.nsec/1000)/1000000.0;
			truth_pose_time = dtos(curr_time);

			drone_flight_logger->truth_pose_addline(truth_pose, truth_pose_time);
			truth_pose_updated   = false;
		}
		if(desired_updated==true){
			ros::Time time = ros::Time::now();
			double curr_time = (double)(time.sec*1e6+time.nsec/1000)/1000000.0;
			desired_time = dtos(curr_time);

			drone_flight_logger->desired_addline(desired_pose, desired_time);
			desired_updated      = false;
		}
		if(vo_pose_updated==true){
			ros::Time time = ros::Time::now();
			double curr_time = (double)(time.sec*1e6+time.nsec/1000)/1000000.0;
			vo_pose_time = dtos(curr_time);

			drone_flight_logger->vo_pose_addline(vo_pose, vo_pose_time);
			vo_pose_updated      = false;
		}
	}
	delete drone_flight_logger;
	return 0;
}
