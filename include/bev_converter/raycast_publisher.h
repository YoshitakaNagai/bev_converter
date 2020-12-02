#ifndef __RAYCAST_PUBLISHER_H
#define __RAYCAST_PUBLISHER_H

#include <ros/ros.h>
// #include <std_msgs/Header>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

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


class RaycastPublisher
{
	public:
		typedef pcl::PointXYZI PointI;
		typedef pcl::PointCloud<PointI> PointCloudI;
		typedef pcl::PointCloud<PointI>::Ptr PointCloudIPtr;
		typedef struct Cast{
			int angle_id;
			float range;
		}CAST;

		RaycastPublisher(void);

		void executor(void);
        void formatter(void);
        void initializer(void);
		void laserscan_callback(const sensor_msgs::LaserScanConstPtr&);
		void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void precast(void);
		int get_angle_id(float&);
		float get_distance(float&, float&);
		void ray_listup(bool);
		void raycast(cv::Mat&);
		std_msgs::Header header_initializer(bool);

	private:
		bool is_first;
		bool laserscan_callback_flag;
		bool pointcloud_callback_flag;
		bool IS_USE_VELODYNE;
		
		std::string FRAME_ID;
        int GRID_NUM;
		int RAY_NUM;
		float angle_resolution;
		float grid_size;
        double WIDTH;
		double Hz;
		double dt;

        ros::NodeHandle nh;
        
		ros::Subscriber laserscan_subscriber;
		ros::Subscriber pointcloud_subscriber;
		ros::Publisher raycast_image_publisher;

		std_msgs::Header velodyne_header;
		sensor_msgs::LaserScan lasers;
		PointCloudIPtr input_points {new PointCloudI};
		// cv::Mat raycast_image32f;
		
		std::vector<Cast> ray_list;
		std::vector<std::vector<Cast> > precast_grid;
		std::vector<std::vector<bool> > is_hit_grid;
};

#endif// __RAYCAST_PUBLISHER_H
