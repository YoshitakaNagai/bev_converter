#ifndef __TEMPORAL_BEV_PUBLISHER_H
#define __TEMPORAL_BEV_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
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
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		PointCloudIPtr pointcloud_transformer(PointCloudIPtr, Eigen::Vector3d, Eigen::Vector3d, tf::Quaternion, tf::Quaternion);
		Eigen::Vector3d displacement_calculator(Eigen::Vector3d, Eigen::Vector3d);
		void bev_generator(PointCloudIPtr, cv::Mat&, int);
		void temporal_bev_generator(cv::Mat&);

	private:
		bool odom_callback_flag;
		bool pointcloud_callback_flag;
		bool is_first;
		bool IS_GAZEBO;
		int GRID_NUM;
		int STEP_MEMORY_SIZE;
		double Hz;
		double WIDTH;
		double grid_resolution;
		double BRIGHTNESS_DECREAS_RATE;
		double ROBOT_RSIZE;
		double current_yaw;

		ros::NodeHandle nh;
		ros::Subscriber obstacle_pointcloud_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher temporal_bev_image_publisher;

		PointCloudIPtr pcl_import_pointcloud {new PointCloudI};
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
