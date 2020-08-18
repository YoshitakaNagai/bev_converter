#ifndef __BEV_IMAGE_GENERATOR_H
#define __BEV_IMAGE_GENERATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"


class BEVImageGenerator
{
	public:
		BEVImageGenerator(double, int);

        typedef struct dynamics{
            double max_acceleration;
            double max_yawrate;
            double max_d_yawrate;
            double max_wheel_angular_velocyty;
            double wheel_radius;
            double tread;
        }Dynamics;
		typedef struct my_odom{
			double x;
			double y;
			double z;
			double roll;
			double pitch;
			double yaw;
		}MyOdom;
        typedef struct oxy{
			Eigen::Vector2i o;
			Eigen::Vector2i x;
			Eigen::Vector2i y;
        }OXY;
		typedef struct unit_vector_oxy{
            OXY src;
            OXY dst;
		}UnitVectorOXY;

		cv::Mat cropped_current_grid_img_generator(cv::Mat);
		cv::Mat cropped_transformed_grid_img_generator(cv::Mat);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
        void formatter(void);
        void initializer(void);
        Eigen::Vector2i cell_motion_calculator(std::string);
		void unit_vector_registrator(void);
        cv::Mat image_transformer(cv::Mat);
        cv::Mat image_cropper(cv::Mat);

	private:
        XmlRpc::XmlRpcValue ROBOT_PARAM;

		static bool odom_callback_flag;
		static bool first_flag;

		constexpr static int Col = 0; //i↓  ...   ↑x
		constexpr static int Row = 1; //j→  ... y←o
        // constexpr int UV_O = 0, int UV_X = 1, int UV_Y = 2; // uv : unit vector
        static std::map<std::string, int> UnitVector;
        static std::map<std::string, int> CropMode;

        int GRID_NUM;
        static int crop_size;
        double RANGE;
        static double Hz, dt, grid_size;

        ros::NodeHandle n;
        ros::NodeHandle nh;
        
		ros::Subscriber grid_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher bev_image_publisher;
		ros::Publisher bev_transformed_image_publisher;
		
        // Eigen::Vector3f zero_vector = Eigen::Vector3f::Zero();

		MyOdom d_my_odom;
        static MyOdom pre_my_odom;
        static MyOdom now_my_odom;
        static Dynamics robot_param;
		static UnitVectorOXY unit_vector;
};

#endif// __BEV_IMAGE_GENERATOR_H
