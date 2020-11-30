#ifndef __RAYCAST_PUBLISHER_H
#define __RAYCAST_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

class RaycastPublisher
{
	public:
		RaycastPublisher(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void laserscan_callback(const sensor_msgs::LaserScanConstPtr&);
		int get_angle_id(float&);
		float get_distance(float&, float&);

	private:
		bool is_first;
		bool laserscan_callback_flag;
		
		std::string FRAME_ID;
        int GRID_NUM;
		int ray_num;
		float angle_resolution;
		float grid_size;
        double RANGE,
		double Hz;
		double dt;

        ros::NodeHandle nh;
        
		ros::Subscriber laserscan_subscriber;
		ros::Publisher raycast_image_publisher;

		sensor_msgs::LaserScan lidar;
		
		typedef struct PRECAST{
			int angle_id;
			double range;
		}Precast;
		std::vector<std::vector<Precast> > precast_grid;
};

#endif// __RAYCAST_PUBLISHER_H
