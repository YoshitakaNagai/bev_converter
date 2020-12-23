#include "bev_converter/temporal_bev_publisher.h"

TemporalBEV::TemporalBEV(void)
: nh("~")
{
	// ros loop rate
	nh.param("Hz", Hz, {100.0});
	// grid
	nh.param("WIDTH", WIDTH, {10.0});
	nh.param("GRID_NUM", GRID_NUM, {50});
	nh.param("BRIGHTNESS_DECREES_RATE", BRIGHTNESS_DECREES_RATE, {0.5});
	// storage
	nh.param("STEP_MEMORY_SIZE", STEP_MEMORY_SIZE, {10});
	// sim or bag
	nh.param("IS_GAZEBO", IS_GAZEBO, {true});

	obstacle_pointcloud_subscriber = nh.subscribe("/velodyne_obstacles", 10, &TemporalBEV::pointcloud_callback, this);
	odom_subscriber = nh.subscribe("/odom", 10, &TemporalBEV::odom_callback, this);

	temporal_bev_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/temporal_bev_image", 10);
}


void TemporalBEV::executor()
{
	formatter();
	Eigen::Vector3d pre_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d d_move = Eigen::Vector3d::Zero();
	tf::Quaternion pre_pose;
	cv::Mat bev_temporal_image = cv::Mat::zeros(image_size, CV_32FC1);

	ros::Rate r(Hz);
	while(ros::ok()){
		if(pointcloud_callback_flag && odom_callback_flag){
			initializer();
			if(is_first){
				pre_position = current_position;
				pre_pose = current_pose;
				is_first = false;
			}

			Eigen::Vector3d d_move = displacement_calculator(pre_position);
			tf::Quaternion relative_rotation = pre_pose * current_pose.inverse();
			relative_rotation.normalize();
			Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
			tf::Quaternion q_global_move(-d_move.x(), -d_move.y(), -d_move.z(), 0.0);
			tf::Quaternion q_local_move = pre_pose.inverse() * q_global_move * pre_pose;
			Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());

			for(int i = 0; i < pointcloud_list.size(); i++){ // Exclude the latest steps
				PointCloudIPtr pcl_tmp_pointcloud{new PointCloudI};
				PointCloudIPtr pcl_transformed_pointcloud{new PointCloudI};
				pcl_tmp_pointcloud = pointcloud_list[i];
				pcl::transformPointCloud(*pcl_tmp_pointcloud, *pcl_transformed_pointcloud, offset, rotation);
				pointcloud_list[i] = pcl_transformed_pointcloud;
				
				int elapsed_step = STEP_MEMORY_SIZE - (i + 1);
				cv::Mat tmp_image = bev_temporal_image.clone();
				bev_generator(pcl_transformed_pointcloud, tmp_image, bev_temporal_image, elapsed_step);
			}

			cv::Mat cv_pub_image32fc1 = bev_temporal_image.clone();
			cv::Mat cv_pub_image8uc1 = cv::Mat::zeros(image_size, CV_32FC1);
			cv_pub_image32fc1.convertTo(cv_pub_image8uc1, CV_8UC1, 255);
			sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_pub_image8uc1).toImageMsg();
			temporal_bev_image_publisher.publish(image_msg);
		}

		r.sleep();
		ros::spinOnce();
	}
}


void TemporalBEV::formatter(void)
{
	pointcloud_callback_flag = false;
	odom_callback_flag = false;
	is_first = true;

	grid_resolution = WIDTH / (double)GRID_NUM;

	pointcloud_list.resize(0);
	image_size = cv::Size(GRID_NUM, GRID_NUM);
}


void TemporalBEV::initializer(void)
{
	pointcloud_callback_flag = false;
	odom_callback_flag = false;
	pointcloud_list.push_back(pcl_import_pointcloud);
	if(pointcloud_list.size() > STEP_MEMORY_SIZE){
		pointcloud_list.erase(pointcloud_list.begin());
	}
}


void TemporalBEV::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "pointcloud_callback" << std::endl;
	sensor_msgs::PointCloud2 import_pointcloud_msg = *msg;
	pcl::fromROSMsg(import_pointcloud_msg, *pcl_import_pointcloud);

	pointcloud_callback_flag = true;
}


void TemporalBEV::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	current_position << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
	quaternionMsgToTF(msg->pose.pose.orientation, current_pose);

	odom_callback_flag = true;
}


Eigen::Vector3d TemporalBEV::displacement_calculator(Eigen::Vector3d previous_position)
{
	Eigen::Vector3d return_d_move = current_position - previous_position;

	return return_d_move;
}


void TemporalBEV::bev_generator(PointCloudIPtr pcl_reference_pointcloud, cv::Mat &src_image, cv::Mat &dst_image, int elapsed_step)
{
	float brightness = std::pow((float)BRIGHTNESS_DECREES_RATE, (float)elapsed_step);
	dst_image = src_image.clone();

	for(auto& pt : pcl_reference_pointcloud->points){
		float robotcs_x = pt.x; 
		float robotcs_y = pt.y; 
		float hit_gridcs_rerow = 0.5 * (float)WIDTH - robotcs_y;
		float hit_gridcs_recol = 0.5 * (float)WIDTH - robotcs_x;
		int hit_row = std::floor(hit_gridcs_rerow / grid_resolution);
		int hit_col = std::floor(hit_gridcs_recol / grid_resolution);
		if((0 < hit_row && hit_row < GRID_NUM) && (0 < hit_col && hit_col < GRID_NUM)){
			if(src_image.at<float>(hit_row, hit_col) <= brightness){
				dst_image.at<float>(hit_row, hit_col) = brightness;
			}else{
				dst_image.at<float>(hit_row, hit_col) = src_image.at<float>(hit_row, hit_col);
			}
		}
	}
}






















