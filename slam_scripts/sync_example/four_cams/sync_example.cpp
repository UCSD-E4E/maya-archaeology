#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// using namespace message_filters;

class Node
{
 public:
  Node()
  {
    sub_1_.subscribe(nh_, "in1", 1);
    sub_2_.subscribe(nh_, "in2", 1);
    sub_3_.subscribe(nh_, "in3", 1);
    sub_4_.subscribe(nh_, "in4", 1);

		pub_1_ = nh_.advertise<sensor_msgs::Image>("out1", 5);
		pub_2_ = nh_.advertise<sensor_msgs::Image>("out2", 5);
		pub_3_ = nh_.advertise<sensor_msgs::Image>("out3", 5);
		pub_4_ = nh_.advertise<sensor_msgs::Image>("out4", 5);

    sync_.reset(new Sync(MySyncPolicy(1000), sub_1_, sub_2_, sub_3_, sub_4_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2, _3, _4));
  }

  void callback(const sensor_msgs::ImageConstPtr &in1, const sensor_msgs::ImageConstPtr &in2, const sensor_msgs::ImageConstPtr &in3, const sensor_msgs::ImageConstPtr &in4)
  {
    ROS_INFO("Synchronization successful");
		pub_1_.publish(in1);
		pub_2_.publish(in2);
		pub_3_.publish(in3);
		pub_4_.publish(in4);
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_1_;
  message_filters::Subscriber<sensor_msgs::Image> sub_2_;
  message_filters::Subscriber<sensor_msgs::Image> sub_3_;
  message_filters::Subscriber<sensor_msgs::Image> sub_4_;
	ros::Publisher pub_1_;
	ros::Publisher pub_2_;
	ros::Publisher pub_3_;
	ros::Publisher pub_4_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer");

  Node synchronizer;

  ros::spin();
}
