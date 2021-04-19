#include <linear_kalman/KalmanEstimator.h>

KalmanEstimator::KalmanEstimator(int rate,  double noise_meas_x, 
											double noise_meas_y, 
											double noise_meas_z, 
											double noise_pos_x, 
											double noise_pos_y, 
											double noise_pos_z, 
											double noise_vel_x, 
											double noise_vel_y, 
											double noise_vel_z,
											double pt1 
											)
	: rate_(rate),
	  new_measurement_(false),
	  start_flag_(false),
	  seq_(0),

	  noise_meas_x_(noise_meas_x),
	  noise_meas_y_(noise_meas_y),
	  noise_meas_z_(noise_meas_z),

	  noise_pos_x_(noise_pos_x),
	  noise_pos_y_(noise_pos_y),
	  noise_pos_z_(noise_pos_z),

	  noise_vel_x_(noise_vel_x),
	  noise_vel_y_(noise_vel_y),
	  noise_vel_z_(noise_vel_z),
	  
	  pt1_(pt1)


{
	transform_input_ = n.subscribe("/predictions", 1, &KalmanEstimator::inputCbRos, this);
	transform_estimated_pub_ = n.advertise<geometry_msgs::PoseStamped>("/flex_sensor/camera_estimate", 1);
	transform_estimated_pub_filtered_ = n.advertise<geometry_msgs::PoseStamped>("/flex_sensor/camera_estimate_filtered", 1);
	
	kf_x_.setMeasureNoise(noise_meas_x_);
	kf_y_.setMeasureNoise(noise_meas_y_);
	kf_z_.setMeasureNoise(noise_meas_z_);

	kf_x_.setPositionNoise(noise_pos_x_);
	kf_y_.setPositionNoise(noise_pos_y_);
	kf_z_.setPositionNoise(noise_pos_z_);

	kf_x_.setVelocityNoise(noise_vel_x_);
	kf_y_.setVelocityNoise(noise_vel_y_);
	kf_z_.setVelocityNoise(noise_vel_z_);


}


void KalmanEstimator::run()
{
	ros::Rate loop_rate(rate_);

	while (ros::ok())
	{
		ros::spinOnce();

		kf_x_.modelUpdate(1.0/rate_);
		kf_y_.modelUpdate(1.0/rate_);
		kf_z_.modelUpdate(1.0/rate_);

		if (new_measurement_)
		{
			new_measurement_ = false;
			kf_x_.measureUpdate(raw_transform_.pose.position.x);
			kf_y_.measureUpdate(raw_transform_.pose.position.y);
			kf_z_.measureUpdate(raw_transform_.pose.position.z);
		}

		geometry_msgs::PoseStamped transform_estimated;

		transform_estimated.header.stamp = ros::Time::now();
		transform_estimated.header.frame_id = raw_transform_.header.frame_id;
		transform_estimated.header.seq = ++seq_; 

		transform_estimated.pose.position.x = kf_x_.getPosition();
		transform_estimated.pose.position.y = kf_y_.getPosition();
		transform_estimated.pose.position.z = kf_z_.getPosition();


		transform_estimated_pub_.publish(transform_estimated);

		kf_x_.pt1Update(pt1_);
		kf_y_.pt1Update(pt1_);
		kf_z_.pt1Update(pt1_);
		

		transform_estimated.pose.position.x = kf_x_.getFiltered();
		transform_estimated.pose.position.y = kf_y_.getFiltered();
		transform_estimated.pose.position.z = kf_z_.getFiltered();


		transform_estimated_pub_filtered_.publish(transform_estimated);
		loop_rate.sleep();
	}
}

void KalmanEstimator::inputCbRos(const geometry_msgs::PoseStamped &msg)
{
	if (!start_flag_)
	{
		start_flag_ = true;
		kf_x_.initializePosition(msg.pose.position.x);
		kf_y_.initializePosition(msg.pose.position.y);
		kf_z_.initializePosition(msg.pose.position.z);
	}
	new_measurement_ = true;
	raw_transform_ = msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KalmanEstimatorNode");
	ros::NodeHandle private_node_handle_("~");

	int rate;
	double noise_meas_x; 
	double noise_meas_y; 
	double noise_meas_z; 
	double noise_pos_x; 
	double noise_pos_y; 
	double noise_pos_z; 
	double noise_vel_x; 
	double noise_vel_y; 
	double noise_vel_z;
	double pt1;

	private_node_handle_.param("rate", rate, int(100));
	private_node_handle_.param("noise_meas_x", noise_meas_x, double(20.));
	private_node_handle_.param("noise_meas_y", noise_meas_y, double(20.));
	private_node_handle_.param("noise_meas_z", noise_meas_z, double(20.));

	private_node_handle_.param("noise_pos_x", noise_pos_x, double(20.));
	private_node_handle_.param("noise_pos_y", noise_pos_y, double(20.));
	private_node_handle_.param("noise_pos_z", noise_pos_z, double(20.));

	private_node_handle_.param("noise_vel_x", noise_vel_x, double(20.));
	private_node_handle_.param("noise_vel_y", noise_vel_y, double(20.));
	private_node_handle_.param("noise_vel_z", noise_vel_z, double(20.));

	private_node_handle_.param("pt1_filter", pt1, double(0.0));

	KalmanEstimator kalman_estimator(rate, 
	noise_meas_x, noise_meas_y, noise_meas_z, 
	noise_pos_x, noise_pos_y, noise_pos_z, 
	noise_vel_x, noise_vel_y, noise_vel_z,
	pt1
	);
	kalman_estimator.run();
}
