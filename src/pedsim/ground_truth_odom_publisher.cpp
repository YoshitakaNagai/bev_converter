#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include "gazebo_msgs/ModelStates.h"

class GroundTruthOdomPublisher
{
	public:
		GroundTruthOdomPublisher(void);

		void gazebo_msg_callback(const gazebo_msgs::ModelStates::ConstPtr&);

	private:
		std::string GAZEBO_MSG_TOPIC, ODOM_TOPIC, FRAME_ID, CHILD_FRAME_ID, MODEL_NAME;
		int step = 0;

		nav_msgs::Odometry odom;
		
		ros::NodeHandle nh;
		ros::Subscriber gazebo_msg_sub;
		ros::Publisher ground_truth_odom_pub;
};


GroundTruthOdomPublisher::GroundTruthOdomPublisher(void)
:nh("~")
{
	nh.param("GAZEBO_MSG_TOPIC", GAZEBO_MSG_TOPIC, {"/gazebo/model_states"});
	nh.param("ODOM_TOPIC", ODOM_TOPIC, {"/ground_truth/odom"});
	nh.param("FRAME_ID", FRAME_ID, {"odom"});
	nh.param("CHILD_FRAME_ID", CHILD_FRAME_ID, {"base_link"});
	nh.param("MODEL_NAME", MODEL_NAME, {"turtlebot3_burger"});
	
	gazebo_msg_sub = nh.subscribe(GAZEBO_MSG_TOPIC, 10, &GroundTruthOdomPublisher::gazebo_msg_callback, this);
	ground_truth_odom_pub = nh.advertise<nav_msgs::Odometry>(ODOM_TOPIC, 10);
}


void GroundTruthOdomPublisher::gazebo_msg_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	odom.header.seq = step;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = FRAME_ID;
	odom.child_frame_id = CHILD_FRAME_ID;

	for(int i = 0; i < msg->name.size(); i++){
		if(msg->name[i] == MODEL_NAME){
			odom.pose.pose = msg->pose[i];
			odom.twist.twist = msg->twist[i];
			break;
		}
	}

	ground_truth_odom_pub.publish(odom);
	step++;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "/ground_truth_odom_publisher");
	
	GroundTruthOdomPublisher ground_truth_odom_publisher;
	ros::spin();

	return 0;
}
