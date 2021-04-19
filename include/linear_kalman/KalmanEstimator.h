#ifndef POZYX_ESTIMATOR
#define POZYX_ESTIMATOR

#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <linear_kalman/KalmanFilter.h>

class KalmanEstimator
{
public:
	KalmanEstimator(int rate, double noise_meas_x, 
											double noise_meas_y, 
											double noise_meas_z, 
											double noise_pos_x, 
											double noise_pos_y, 
											double noise_pos_z, 
											double noise_vel_x, 
											double noise_vel_y, 
											double noise_vel_z,
											double pt1);
	void inputCbRos(const geometry_msgs::PoseStamped &msg);
	void run(void);
private:
	ros::NodeHandle n;
	ros::Subscriber transform_input_;
	ros::Publisher transform_estimated_pub_, transform_estimated_pub_filtered_;
	int rate_, seq_;
	bool new_measurement_, start_flag_;

	double noise_meas_x_, noise_meas_y_, noise_meas_z_;
	double noise_pos_x_, noise_pos_y_, noise_pos_z_;
	double noise_vel_x_, noise_vel_y_, noise_vel_z_;

	double pt1_;
	KalmanFilter kf_x_, kf_y_, kf_z_;

	geometry_msgs::PoseStamped raw_transform_;
};

#endif