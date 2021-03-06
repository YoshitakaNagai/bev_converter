#ifndef __TEMPORAL_BEV_PUBLISHER_H
#define __TEMPORAL_BEV_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"


class TemporalBEV
{
	public:
		typedef pcl::PointXYZI PointI;
		typedef pcl::PointCloud<PointI> PointCloudI;
		typedef pcl::PointCloud<PointI>::Ptr PointCloudIPtr;

		TemporalBEV(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void scan_callback(const sensor_msgs::LaserScanConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void episode_flag_callback(const std_msgs::Bool::ConstPtr&);
		PointCloudIPtr pointcloud_transformer(PointCloudIPtr, Eigen::Vector3d, Eigen::Vector3d, tf::Quaternion, tf::Quaternion);
		Eigen::Vector3d displacement_calculator(Eigen::Vector3d, Eigen::Vector3d);
		void bev_generator(PointCloudIPtr, cv::Mat&, int);
		void temporal_bev_generator(cv::Mat&);

	private:
		bool odom_callback_flag;
		bool pointcloud_callback_flag;
		bool scan_callback_flag;
		bool episode_flag_callback_flag;
		bool is_first;
		bool is_finish_episode;
		bool IS_GAZEBO;
		bool IS_USE_2D_LIDAR;
		int GRID_NUM;
		int STEP_MEMORY_SIZE;
		double Hz;
		double WIDTH;
		double grid_resolution;
		double BRIGHTNESS_DECREAS_RATE;
		double ROBOT_RSIZE;
		double SCAN_ERROR_THRESHOLD;
		double current_yaw;

		ros::NodeHandle nh;
		ros::Subscriber obstacle_pointcloud_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Subscriber scan_subscriber;
		ros::Subscriber episode_flag_subscriber;
		ros::Publisher temporal_bev_image_publisher;

		sensor_msgs::LaserScan scan_msg;

		PointCloudIPtr pcl_import_pointcloud {new PointCloudI};
		PointCloudIPtr pcl_process_pointcloud {new PointCloudI};
		PointCloudIPtr pcl_2d_scan_bfr {new PointCloudI};
		std::vector<PointCloudIPtr> pointcloud_list;


		cv::Size image_size;
		cv::Mat format_image;
		std::vector<cv::Mat> image_list;

		tf::Quaternion current_pose;
		tf::Quaternion previous_pose;
		Eigen::Vector3d current_position;
		Eigen::Vector3d previous_position;
};

#endif// __TEMPORAL_BEV_PUBLISHER_H
