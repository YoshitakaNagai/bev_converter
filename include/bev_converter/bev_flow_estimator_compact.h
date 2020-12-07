#ifndef __BEV_FLOW_ESTIMATOR_COMPACT_H
#define __BEV_FLOW_ESTIMATOR_COMPACT_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

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


class BEVFlowEstimator
{
	public:
		typedef pcl::PointXYZI PointI;
		typedef pcl::PointCloud<PointI> PointCloudI;
		typedef pcl::PointCloud<PointI>::Ptr PointCloudIPtr;

		BEVFlowEstimator(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void dynamic_pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void cuurent_dynamic_image_generator(cv::Mat&);
        bool flow_estimator(cv::Mat&, cv::Mat&, cv::Mat&, std_msgs::Float32MultiArray&, std_msgs::Float32MultiArray&);
		void image_cropper(cv::Mat&, cv::Mat&);
		void image_transformer(cv::Mat&, cv::Mat&, Eigen::Vector3d, Eigen::Vector3d, double, double);

	private:
        XmlRpc::XmlRpcValue ROBOT_PARAM;

		bool odom_callback_flag;
		bool dynamic_pointcloud_callback_flag;
		bool tf_listen_flag;
		bool flow_flag;
        bool IS_SPARCE;
        bool IS_DRAW_FLOW_LINE;
		bool IS_GAZEBO;
		bool is_first;
		
		std::string PKG_PATH, FRAME_ID, CHILD_FRAME_ID, CMD_VEL_TOPIC;
        int GRID_NUM, FLOW_WINiDOW_SIZE, MANUAL_CROP_SIZE, MAX_CORNERS, WIN_SIZE, MAX_COUNT;
		int step, bev_seq, cropped_grid_num;
        double WIDTH, Hz, grid_size, dt, QUALITY_LEVEL, MIN_DISTANCE;
		double MAX_HUMAN_VELOCITY, MIN_HUMAN_VELOCITY;
    	double run_length;
		double current_yaw, previous_yaw;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber dynamic_pointcloud_subscriber;
		ros::Subscriber odom_subscriber;

		ros::Publisher flow_image_publisher;
		// for debug
		ros::Publisher previous_image_publisher;
		ros::Publisher current_image_publisher;

		ros::Publisher flow_multiarray_x_publisher;
		ros::Publisher flow_multiarray_y_publisher;
		
		std_msgs::Float32MultiArray flow_multiarray_x, flow_multiarray_y;
		nav_msgs::Odometry odom;

		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener listener_;

		PointCloudIPtr pcl_import_pointcloud {new PointCloudI};

		cv::Mat current_dynamic_image;
		cv::Mat previous_dynamic_image;
		cv::Mat transformed_dynamic_image;
		cv::Mat cropped_current_dynamic_image;
		cv::Mat cropped_transformed_dynamic_image;


		Eigen::Vector3d current_position;
		Eigen::Vector3d previous_position;
		Eigen::Affine3d affine_transform;
		pcl::PointXYZ pt0, pt1, pt2;
		pcl::PointCloud<pcl::PointXYZ> src_euqlid_3pts;
		pcl::PointCloud<pcl::PointXYZ> dst_euqlid_3pts;
};

#endif// __BEV_FLOW_ESTIMATOR_COMPACT_H
