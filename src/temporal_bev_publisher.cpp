#include "bev_converter/temporal_bev_publisher.h"

TemporalBEV::TemporalBEV(void)
: nh("~")
{
	// ros loop rate
	nh.param("Hz", Hz, {100.0});
	// grid
	nh.param("WIDTH", WIDTH, {10.0});
	nh.param("GRID_NUM", GRID_NUM, {50});
	nh.param("BRIGHTNESS_DECREAS_RATE", BRIGHTNESS_DECREAS_RATE, {0.5});
	// storage
	nh.param("STEP_MEMORY_SIZE", STEP_MEMORY_SIZE, {10});
	// sim or bag
	nh.param("IS_GAZEBO", IS_GAZEBO, {true});
	// robot
	nh.param("ROBOT_RSIZE", ROBOT_RSIZE, {0.13});
	nh.param("SCAN_ERROR_THRESHOLD", SCAN_ERROR_THRESHOLD, {1.0});
	nh.param("IS_USE_2D_LIDAR", IS_USE_2D_LIDAR, {true});

	obstacle_pointcloud_subscriber = nh.subscribe("/velodyne_obstacles", 10, &TemporalBEV::pointcloud_callback, this);
	scan_subscriber = nh.subscribe("/scan", 10, &TemporalBEV::scan_callback, this);
	odom_subscriber = nh.subscribe("/odom", 10, &TemporalBEV::odom_callback, this);
	episode_flag_subscriber = nh.subscribe("/is_start_episode", 10, &TemporalBEV::episode_flag_callback, this);

	temporal_bev_image_publisher = nh.advertise<sensor_msgs::Image>("/bev/temporal_bev_image", 10);
}


void TemporalBEV::executor()
{
	formatter();
	Eigen::Vector3d now_position = Eigen::Vector3d::Zero();
	Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
	tf::Quaternion now_pose;
	tf::Quaternion last_pose;

	ros::Rate r(Hz);
	while(ros::ok()){
		if(pointcloud_callback_flag && odom_callback_flag){
			initializer();
			
			now_position = current_position;
			last_position = previous_position;
			now_pose = current_pose;
			last_pose = previous_pose;

			for(int i = 0; i < pointcloud_list.size(); i++){ // Exclude the latest steps
				PointCloudIPtr pcl_tmp_pointcloud{new PointCloudI};
				PointCloudIPtr pcl_transformed_pointcloud{new PointCloudI};
				pcl_tmp_pointcloud = pointcloud_list[i];
				pcl_transformed_pointcloud = pointcloud_transformer(pcl_tmp_pointcloud, now_position, last_position, now_pose, last_pose);
				pointcloud_list[i] = pcl_transformed_pointcloud;
				
				int elapsed_step = pointcloud_list.size() - (i + 1);
				cv::Mat bev_image = cv::Mat::zeros(image_size, CV_32FC1);
				bev_generator(pcl_transformed_pointcloud, bev_image, elapsed_step);
				image_list[i] = bev_image.clone();
			}
			
			cv::Mat bev_temporal_image = cv::Mat::zeros(image_size, CV_32FC1);
			temporal_bev_generator(bev_temporal_image);

			cv::Mat cv_pub_image32fc1 = bev_temporal_image.clone();
			cv::Mat cv_pub_image8uc1 = cv::Mat::zeros(image_size, CV_32FC1);
			cv_pub_image32fc1.convertTo(cv_pub_image8uc1, CV_8UC1, 255);
			sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cv_pub_image8uc1).toImageMsg();
			temporal_bev_image_publisher.publish(image_msg);

			previous_position = now_position;
			previous_pose = now_pose;
		}

		r.sleep();
		ros::spinOnce();
	}
}


void TemporalBEV::formatter(void)
{
	pointcloud_callback_flag = false;
	scan_callback_flag = false;
	odom_callback_flag = false;
	episode_flag_callback_flag = false;
	is_first = true;

	grid_resolution = WIDTH / (double)GRID_NUM;
	image_size = cv::Size(GRID_NUM, GRID_NUM);
	format_image = cv::Mat::zeros(image_size, CV_32FC1);

	pointcloud_list.resize(0);
	pcl_2d_scan_bfr->points.resize(0);

	image_list.resize(0);
	for(int i = 0; i < STEP_MEMORY_SIZE; i++){
		image_list.push_back(format_image);
	}
}


void TemporalBEV::initializer(void)
{
	if(is_first){
		previous_position = current_position;
		previous_pose = current_pose;
		is_first = false;
	}



	if(episode_flag_callback_flag){
		std::cout << "is_finish_episode : " << is_finish_episode << std::endl;
		if(is_finish_episode){
			pointcloud_list.clear();
			std::cout << "clear!!" << std::endl;

			for(int i = 0; i < STEP_MEMORY_SIZE; i++){
				image_list[i] = format_image.clone();
			}
		}
	}

	
	// if(image_list.size() > STEP_MEMORY_SIZE){
	// 	image_list.erase(image_list.begin());
	// }
	std::cout << "image_list.size() = " << image_list.size() << std::endl;

	pcl_process_pointcloud = pcl_import_pointcloud;

	if(IS_USE_2D_LIDAR){
		if(scan_callback_flag){
			PointCloudIPtr pcl_2d_scan{new PointCloudI};
			pcl_2d_scan->points.resize(0);
			for(int i = 0; i < scan_msg.ranges.size(); i++){
				if(isfinite(scan_msg.ranges[i])){
					PointI laser_point;
					laser_point.x = scan_msg.ranges[i] * cos(i * scan_msg.angle_increment);
					laser_point.y = scan_msg.ranges[i] * sin(i * scan_msg.angle_increment);
					laser_point.z = 0.18;
					laser_point.intensity = scan_msg.intensities[i];
					pcl_2d_scan->points.push_back(laser_point);
				}
			}
			int pcl_size = 0;
			if(pcl_2d_scan->points.size() <= pcl_2d_scan_bfr->points.size()){
				pcl_size = pcl_2d_scan->points.size();
			}else{
				pcl_size = pcl_2d_scan_bfr->points.size();
			}
			float error_sum = 0.0;
			for(int i = 0; i < pcl_size; i++){
				float x = pcl_2d_scan->points[i].x;
				float y = pcl_2d_scan->points[i].y;
				float x_bfr = pcl_2d_scan_bfr->points[i].x;
				float y_bfr = pcl_2d_scan_bfr->points[i].y;
				float range = sqrt(x * x + y * y);
				float range_bfr = sqrt(x_bfr * x_bfr + y_bfr * y_bfr);
				error_sum += std::fabs(range - range_bfr);
			}

			if(error_sum < SCAN_ERROR_THRESHOLD){
				*pcl_process_pointcloud += *pcl_2d_scan;
			}
			pcl_2d_scan_bfr = pcl_2d_scan;
		}
	}

	pointcloud_list.push_back(pcl_process_pointcloud);
	if(pointcloud_list.size() > STEP_MEMORY_SIZE){
		pointcloud_list.erase(pointcloud_list.begin());
	}
	std::cout << "pointcloud_list.size() = " << pointcloud_list.size() << std::endl;

	pointcloud_callback_flag = false;
	scan_callback_flag = false;
	odom_callback_flag = false;
	episode_flag_callback_flag = false;
}


void TemporalBEV::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "pointcloud_callback" << std::endl;
	sensor_msgs::PointCloud2 import_pointcloud_msg = *msg;
	pcl::fromROSMsg(import_pointcloud_msg, *pcl_import_pointcloud);

	pointcloud_callback_flag = true;
}


void TemporalBEV::scan_callback(const sensor_msgs::LaserScanConstPtr &msg)
{
	// std::cout << "pointcloud_callback" << std::endl;
	scan_msg = *msg;

	scan_callback_flag = true;
}


void TemporalBEV::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	current_position << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
	quaternionMsgToTF(msg->pose.pose.orientation, current_pose);

	odom_callback_flag = true;
}


void TemporalBEV::episode_flag_callback(const std_msgs::Bool::ConstPtr &msg)
{
	is_finish_episode = msg->data;

	episode_flag_callback_flag = true;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr TemporalBEV::pointcloud_transformer(PointCloudIPtr pcl_reference_pointcloud,
																		 Eigen::Vector3d now_position,
																		 Eigen::Vector3d last_position,
																		 tf::Quaternion now_pose,
																		 tf::Quaternion last_pose)
{
	PointCloudIPtr pcl_return_pointcloud{new PointCloudI};

	Eigen::Vector3d d_move = displacement_calculator(now_position, last_position);
	tf::Quaternion relative_rotation = last_pose * now_pose.inverse();
	relative_rotation.normalize();
	Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
	tf::Quaternion q_global_move(-d_move.x(), -d_move.y(), -d_move.z(), 0.0);
	tf::Quaternion q_local_move = last_pose.inverse() * q_global_move * last_pose;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());

	pcl::transformPointCloud(*pcl_reference_pointcloud, *pcl_return_pointcloud, offset, rotation);

	return pcl_return_pointcloud;
}


Eigen::Vector3d TemporalBEV::displacement_calculator(Eigen::Vector3d now_position, Eigen::Vector3d last_position)
{
	Eigen::Vector3d return_d_move = now_position - last_position;

	return return_d_move;
}


void TemporalBEV::bev_generator(PointCloudIPtr pcl_reference_pointcloud, cv::Mat &dst_image, int elapsed_step)
{
	float brightness = std::pow((float)BRIGHTNESS_DECREAS_RATE, elapsed_step);

	for(auto& pt : pcl_reference_pointcloud->points){
		float robotcs_x = pt.x; 
		float robotcs_y = pt.y; 
		float hit_gridcs_rerow = 0.5 * (float)WIDTH - robotcs_y;
		float hit_gridcs_recol = 0.5 * (float)WIDTH - robotcs_x;
		int hit_row = std::floor(hit_gridcs_rerow / grid_resolution);
		int hit_col = std::floor(hit_gridcs_recol / grid_resolution);
		if((0 < hit_row && hit_row < GRID_NUM) && (0 < hit_col && hit_col < GRID_NUM)){
			dst_image.at<float>(hit_row, hit_col) = brightness;
		}
	}
}


void TemporalBEV::temporal_bev_generator(cv::Mat &dst_image)
{
	for(int col = 0; col < GRID_NUM; col++){
		for(int row = 0; row < GRID_NUM; row++){
			float max_brightness = 0.0;
			for(int i = 0; i < image_list.size(); i++){
				cv::Mat tmp_image = cv::Mat::zeros(image_size, CV_32FC1);
				cv::Mat reference_image = cv::Mat::zeros(image_size, CV_32FC1);
				tmp_image = image_list[i];
				reference_image = tmp_image.clone();
				float reference_brightness = reference_image.at<float>(row, col);
				if(reference_brightness > max_brightness){
					max_brightness = reference_brightness;
				}
			}
			dst_image.at<float>(row, col) = max_brightness;

			float distance_from_center_x = 0.5 * (float)WIDTH - grid_resolution * col;
			float distance_from_center_y = 0.5 * (float)WIDTH - grid_resolution * row;
			float distance_from_center_x_pow = distance_from_center_x * distance_from_center_x;
			float distance_from_center_y_pow = distance_from_center_y * distance_from_center_y;
			float distance_from_center = sqrt(distance_from_center_x_pow + distance_from_center_y_pow);
			if(distance_from_center < ROBOT_RSIZE){
				dst_image.at<float>(row, col) = 1.0;
			}
		}
	}
}



