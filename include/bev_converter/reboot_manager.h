#ifndef __REBOOT_MANAGER_H
#define __REBOOT_MANAGER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <unistd.h>
#include <signal.h>

class RebootManager
{
	public:
		RebootManager(void);

		void executor(void);
		void formatter(void);
		void initializer(void);
		void reboot(void);
		void flow_image_callback(const sensor_msgs::ImageConstPtr&);
		pid_t roslauncher(const std::string&, const std::string&);

	private:
		bool is_first;
		bool flow_image_callback_flag;
		int READ;
		int WRITE;
		int COUNT_TIME;
		double callback_time;
		double Hz;
		double BORDER_TIME;
		std::string LAUNCH_PATH;
		std::string BIN_PATH;

		pid_t pid_gazebo_bev_flow_estimator;
		
		ros::NodeHandle nh;
		ros::Subscriber flow_image_subscriber;
};

#endif// __BEV_FLOW_ESTIMATOR_H
