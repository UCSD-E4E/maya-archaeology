#include "ros/ros.h"
#include "sensor_msgs/Imu.h"


class SubscribeAndPublish
{
	public:
		SubscribeAndPublish(const char* sub_topic, const char* pub_topic):
			msg_sent(true)
		{
			pub_ = n_.advertise<sensor_msgs::Imu>(pub_topic, 1);

			sub_ = n_.subscribe(sub_topic, 1, &SubscribeAndPublish::callback, this);

			ROS_INFO("IMU resample node initialized.");
		}

		void callback(const sensor_msgs::Imu::ConstPtr& input)
		{
			if(input->header.stamp < msg.header.stamp){ ROS_WARN("Dropping IMU data"); return; };
			if(msg_sent){ msg = *input; msg_sent = false; }
			else
			{
				if(input->linear_acceleration_covariance[0] != -1)
				{
					msg.linear_acceleration = input->linear_acceleration;
					msg.linear_acceleration_covariance = input->linear_acceleration_covariance; // copy a boost::array
				}
				if(input->angular_velocity_covariance[0] != -1)
				{
					msg.angular_velocity = input->angular_velocity;
					msg.angular_velocity_covariance = input->angular_velocity_covariance; // copy a boost::array
				}
			}
			
			if(msg.angular_velocity_covariance[0] != -1 && msg.linear_acceleration_covariance[0] != -1)
			{
				pub_.publish(msg);
				msg_sent = true;
			}
		}

	private:
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Subscriber sub_;
		sensor_msgs::Imu msg;
		bool msg_sent;

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_listener");

	SubscribeAndPublish task("/camera/imu/data_raw", "/imu");

	ros::spin();

	return 0;
}
